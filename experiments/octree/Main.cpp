
#include "experiments/common/IApplication.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <string>
#include <Precompiled.hpp>

// Random numbor between 0 and 1
float frand()
{
	return float( rand() & 32767 ) / 32767.0f;
};
// Random numbor between -1 and 1
float crand()
{
	return frand() * 2.0f - 1.0f;
};
// Random vector between min and max
adm::Vec3 randVec( adm::Vec3 min, adm::Vec3 max )
{
	const adm::Vec3 centre = (min + max) * 0.5f;
	const adm::Vec3 extent = max - centre;
	return centre + adm::Vec3(
		crand() * extent.x * 0.9f,
		crand() * extent.y * 0.9f,
		crand() * extent.z * 0.9f
	);
};

class OctreeExperiment : public IApplication
{
public:
	bool Init() override
	{
		using namespace adm;

		viewMatrix = glm::identity<glm::mat4>();
		projectionMatrix = glm::perspective( glm::radians( 90.0f ), 16.0f / 9.0f, 0.01f, 1024.0f );
		viewProjectionMatrix = viewMatrix;

		// 20x20x20 units
		const AABB octreeBox = { Vec3( 0.0f ), Vec3( 20.0f ) };

		const auto shouldSubdivide = []( const adm::Octree<adm::Vec3>::NodeType& node )
		{	// This is our threshold. If there's more than 50 elements inside a node,
			// it'll be subdivided
			//return node.GetNumElements() > 50;
			
			// Alternative heuristic: reverse density and distance from centre
			const int32_t& numElements = node.GetNumElements();
			
			const adm::Vec3 nodeCentre = node.GetBoundingVolume().GetCentre();
			adm::Vec3 averageCentre;
			node.ForEachElement( [&]( adm::Vec3* element )
				{
					averageCentre += *element;
				} );
			averageCentre /= float( numElements );

			const float diagonal = node.GetBoundingVolume().Diagonal();
			const float density = float( numElements ) / diagonal;
			const float relativeDistanceFromCentre = (nodeCentre - averageCentre).Length() / (diagonal * 0.5f);

			if ( numElements > 500 )
			{
				return true;
			}
			if ( diagonal < 4.0f )
			{
				return false;
			}

			return relativeDistanceFromCentre > 0.3f || density < 0.6f;
		};

		octree.Initialise( octreeBox,
			adm::utils::IntersectsAABB,
			adm::utils::OccupiesBox,
			//shouldSubdivide,
			adm::utils::SimpleThreshold<adm::Vec3, 40>,
			adm::utils::GetAABBForChild );

		srand( 0x910583 );

		const auto canSpawnHere = []( const Vec3& point ) -> bool
		{
			// The closer the point is to 0,0,0, the less chance it'll spawn
			const float threshold = 10.0f + frand() * 8.0f;
			// Similarly there's another disc out there
			const float otherThreshold = 30.0f + frand() * 5.0f;
			const float pointDistance = point.Length();
			
			return pointDistance > threshold 
				&& std::abs(otherThreshold - pointDistance) > 10.0f
				&& point.z < 7.0f + frand() * 10.0f;
		};

		adm::Timer timer;

		constexpr int numPoints = 2500;
		Vector<Vec3> points;
		points.reserve( numPoints );
		for ( int i = 0; i < numPoints; i++ )
		{
			Vec3 point = randVec( octreeBox.mins, octreeBox.maxs );
			if ( canSpawnHere( point ) )
			{
				points.push_back( point );
			}
			else
			{
				i--;
			}
		}

		octree.SetElements( std::move( points ) );

		float spawningMs = timer.GetElapsedAndReset();

		octree.Rebuild();

		float buildingMs = timer.GetElapsed();

		std::cout << "Took " << spawningMs << " ms to populate, " << buildingMs << " ms to build the octree" << std::endl;

		return true;
	}

	void Shutdown() override
	{

	}

	void UpdateViewMatrix()
	{
		using namespace glm;

		// Spherical coords
		const vec3 anglesr = radians( angles );

		const float cosPitch = cos( anglesr.x );
		const float sinPitch = sin( anglesr.x );
		const float cosYaw = cos( anglesr.y );
		const float sinYaw = sin( anglesr.y );
		const float cosRoll = cos( anglesr.z );
		const float sinRoll = sin( anglesr.z );

		viewForward =
		{
			cosYaw * cosPitch,
			-sinYaw * cosPitch,
			-sinPitch
		};

		//std::cout << "viewForward: " << viewForward.x << " "
		//	<< viewForward.y << " " << viewForward.z << std::endl;

		viewUp =
		{
			(cosRoll * sinPitch * cosYaw) + (-sinRoll * -sinYaw),
			(cosRoll * -sinPitch * sinYaw) + (-sinRoll * cosYaw),
			cosPitch * cosRoll
		};

		viewRight = normalize( cross( viewForward, viewUp ) );

		// glm::lookAt does this but in a slightly more convoluted way
		// So let's just do it ourselves
		viewMatrix[0][0] = viewRight.x;
		viewMatrix[1][0] = viewRight.y;
		viewMatrix[2][0] = viewRight.z;

		viewMatrix[0][1] = viewUp.x;
		viewMatrix[1][1] = viewUp.y;
		viewMatrix[2][1] = viewUp.z;

		viewMatrix[0][2] = -viewForward.x;
		viewMatrix[1][2] = -viewForward.y;
		viewMatrix[2][2] = -viewForward.z;

		viewMatrix[3][0] = -glm::dot( viewRight, position );
		viewMatrix[3][1] = -glm::dot( viewUp, position );
		viewMatrix[3][2] = glm::dot( viewForward, position );

		viewMatrix[0][3] = 1.0f;
		viewMatrix[1][3] = 1.0f;
		viewMatrix[2][3] = 1.0f;
		viewMatrix[3][3] = 1.0f;

		viewProjectionMatrix = projectionMatrix * viewMatrix;
	}

	void Render( const float& deltaTime )
	{
		const auto renderText = [&]( adm::Vec3 textPosition, adm::StringView text )
		{
			float distance = std::max( 1.0f, (adm::Vec3( &position.x ) - textPosition).Length() );
			if ( distance > 10.0f )
			{
				return;
			}

			dd::projectedText( text.data(), textPosition, dd::colors::White, &viewProjectionMatrix[0][0], 0, 0, 1600, 900, 20.0f / (distance * distance) );
		};

		const auto renderBbox = []( adm::AABB bbox, adm::Vec3 colour = { 1.0f, 1.0f, 1.0f })
		{
			adm::Vec3 centre = bbox.GetCentre();
			adm::Vec3 extents = bbox.GetExtents() * 1.98f;

			dd::box( centre, colour, extents.x, extents.y, extents.z );
		};

		const auto generateColour = []() -> adm::Vec3
		{
			return adm::Vec3(
				(0.5f + crand() * 0.4f)/* * frand() */,
				(0.5f + crand() * 0.4f)/* * frand() */,
				(0.5f + crand() * 0.4f)/* * frand() */
			).Normalized();
		};

		srand( 0x24819 );
		int nodeId = 0;
		constexpr float boxSize = 0.06f;
		for ( auto& node : octree.GetLeaves() )
		{
			adm::Vec3 sectorColour = generateColour();

			renderBbox( node->GetBoundingVolume(), sectorColour );
			//renderText( node->GetBoundingBox().GetCentre(), std::to_string( nodeId ) );
		
			node->ForEachElement( [&]( adm::Vec3* point )
				{
					//dd::box( *point, sectorColour, boxSize, boxSize, boxSize );
					dd::point( *point, sectorColour, 2.0f );
				} );

			nodeId++;
		}

		const ddVec3 textPosition = { 20.0f, 20.0f, 0.0f };
		std::string framerate = "Elements: " + std::to_string( -octree.GetNodes().front().GetNumElements() ) + ", fps: ";
		framerate += std::to_string( 1.0f / deltaTime );
		
		dd::screenText( framerate.c_str(), textPosition, dd::colors::White, 1.0f );
	}

	void Update( const float& deltaTime, const float& time, const UserCommand& uc ) override
	{
		position += uc.forward * viewForward * deltaTime * 3.0f + uc.right * viewRight * deltaTime * 3.0f;

		if ( uc.flags & UserCommand::Action1 )
		{
			angles.y += uc.mouseX * 0.16f;
			angles.x += uc.mouseY * 0.16f;
		}

		UpdateViewMatrix();
		Render( deltaTime );
	}

	const float* GetViewProjectionMatrix() const
	{
		return &viewProjectionMatrix[0][0];
	}

private:
	adm::NTree<adm::Vec3, adm::AABB, 3> octree;

	glm::vec3 position{ 0.0f, 0.0f, 0.0f };
	glm::vec3 angles{ 0.0f, 0.0f, 0.0f };

	glm::vec3 viewForward{ 1.0f, 0.0f, 0.0f };
	glm::vec3 viewRight{ 0.0f, -1.0f, 0.0f };
	glm::vec3 viewUp{ 0.0f, 0.0f, 1.0f };

	glm::mat4 viewMatrix;
	glm::mat4 projectionMatrix;
	glm::mat4 viewProjectionMatrix;
};

DeclareExperiment( OctreeExperiment );
