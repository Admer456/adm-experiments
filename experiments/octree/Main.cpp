
#include "experiments/common/IApplication.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <string>
#include <Precompiled.hpp>

namespace adm
{
	// Min-max axis-aligned bounding box
	class AABB
	{
	public: // Constructors
		AABB() = default;
		AABB( const AABB& bbox ) = default;
		AABB( AABB&& bbox ) = default;
		AABB( Vec3 min, Vec3 max )
			: mins( min ), maxs( max )
		{
			if ( IsInverted() )
			{
				Fix();
			}
		}
		AABB( const Vector<Vec3>& points )
		{
			for ( const auto& point : points )
			{
				Add( point );
			}
		}

	public: // Methods
		// Expands the bbox if the point is outside of it
		inline void Add( const Vec3& point )
		{
			mins.x = std::min( mins.x, point.x );
			mins.y = std::min( mins.y, point.y );
			mins.z = std::min( mins.z, point.z );
			maxs.x = std::max( maxs.x, point.x );
			maxs.y = std::max( maxs.y, point.y );
			maxs.z = std::max( maxs.z, point.z );
		}

		inline void Fix()
		{
			if ( mins.x > maxs.x )
			{
				std::swap( mins.x, maxs.x );
			}
			if ( mins.y > maxs.y )
			{
				std::swap( mins.y, maxs.y );
			}
			if ( mins.z > maxs.z )
			{
				std::swap( mins.z, maxs.z );
			}
		}

		// Checks if a point is inside the bounding box
		inline bool IsInside( Vec3 point ) const
		{
			return point.x >= mins.x && point.y >= mins.y && point.z >= mins.z
				&& point.x <= maxs.x && point.y <= maxs.y && point.z <= maxs.z;
		}

		// Length of the 3D diagonal from mins to maxs
		inline float Diagonal() const
		{
			return (mins - maxs).Length();
		}

		// Checks if mins and maxs accidentally swapped places
		inline bool IsInverted() const
		{
			return mins.x > maxs.x || mins.y > maxs.y || mins.z > maxs.z;
		}

		// Gets the centre point between mins and maxs
		Vec3 GetCentre() const
		{
			return (mins + maxs) * 0.5f;
		}

		// Gets the extents of the box from its centre
		Vec3 GetExtents() const
		{
			return maxs - GetCentre();
		}

		// Forms a box from mins and maxs and gets all the vertices
		// Vertices are arranged as a top & bottom face, clockwise order
		Vector<Vec3> GetBoxPoints() const
		{
			return 
			{
				Vec3( mins.x, mins.y, maxs.z ),
				Vec3( mins.x, maxs.y, maxs.z ),
				maxs,
				Vec3( maxs.x, mins.y, maxs.z ),

				Vec3( maxs.x, maxs.y, mins.z ),
				Vec3( maxs.x, mins.y, mins.z ),
				mins,
				Vec3( mins.x, maxs.y, mins.z ),
			};
		}

	public: // Operators
		AABB operator+( const AABB& bbox ) const
		{
			return AABB( *this ) += bbox;
		}

		AABB& operator+=( const AABB& bbox )
		{
			Add( bbox.mins );
			Add( bbox.maxs );
			return *this;
		}

		AABB& operator=( const AABB& bbox ) = default;
		AABB& operator=( AABB&& bbox ) = default;
		bool operator==( const AABB& bbox ) const
		{
			return mins == bbox.mins && maxs == bbox.maxs;
		}

	public: // Member variables
		Vec3 mins{ Vec3::Zero };
		Vec3 maxs{ Vec3::Zero };
	};

	// Non-copyable octree node
	template<typename elementType>
	struct Node
	{
		using ForEachChildFn = void( Node* );
		using ForEachElementFn = void( elementType );

		Node() = default;
		Node( const Node& node ) = delete;
		Node( Node&& node ) = default;
		Node& operator=( Node&& node ) = default;

		Node( const AABB& bbox )
			: bbox( bbox ), numElements( 0 )
		{
		}

		Node( const AABB& bbox, const Vector<elementType>& elementList )
			: bbox( bbox )
		{
			numElements = elementList.size();
			for ( const auto& element : elementList )
			{
				elements.push_back( element );
			}
		}

		Node( const AABB& bbox, const LinkedList<elementType>& elementList )
			: bbox( bbox ), elements( elementList )
		{
			for ( const auto& element : elements )
			{
				numElements++;
			}
		}

		void AddElement( elementType element )
		{
			elements.push_back( element );
			numElements++;
		}

		void CreateChildren( LinkedList<Node<elementType>>& octreeNodes )
		{
			const Vec3 sizes[]
			{
				bbox.mins, bbox.maxs
			};

			// How to interpret this:
			// 000 -> mins.x, mins.y, mins.z
			// 010 -> mins.x, maxs.y, mins.z
			// The number basically means which extent to take the component from
			const int sizeIndices[]
			{
				0, 0, 0,
				0, 0, 1,
				0, 1, 0,
				0, 1, 1,
				1, 0, 0,
				1, 0, 1,
				1, 1, 0,
				1, 1, 1,
			};

			const auto getBboxForChildNode = [&]( int i ) -> AABB
			{
				Vec3 centre = bbox.GetCentre();
				Vec3 extent
				{
					sizes[sizeIndices[i * 3 + 0]].x,
					sizes[sizeIndices[i * 3 + 1]].y,
					sizes[sizeIndices[i * 3 + 2]].z
				};

				// AABB will be swapped if it's inverted, so worry not
				return AABB( centre, extent );
			};

			for ( uint32_t i = 0; i < 8; i++ )
			{
				auto& child = children[i];
				child = &octreeNodes.emplace_back( getBboxForChildNode( i ) );
			}

			// Become a non-leaf because you have children now
			if ( IsLeaf() )
			{
				numElements *= -1;
			}
		}

		bool IsLeaf() const
		{
			return numElements > 0;
		}

		bool IsEmpty() const
		{
			return numElements == 0;
		}

		const AABB& GetBoundingBox() const
		{
			return bbox;
		}

		int32_t GetNumElements() const
		{
			return numElements;
		}

		void ForEachChild( std::function<ForEachChildFn> function ) const
		{
			if ( IsLeaf() )
			{
				return;
			}

			for ( auto& child : children )
			{
				function( child );
			}
		}

		void ForEachElement( std::function<ForEachElementFn> function ) const
		{
			for ( auto& element : elements )
			{
				function( element );
			}
		}

	private:
		AABB bbox{};
		// > 0 -> leaf
		// = 0 -> empty
		// < 0 -> node with children
		int32_t numElements{ 0 };
		LinkedList<elementType> elements;

		Node* children[8]{};
	};

	// Non-copyable octree designed to host static elements
	template<typename elementType>
	class OctreeStatic
	{
	public:
		using NodeType = Node<elementType*>;

		// Does the element intersect an AABB?
		using IntersectsBoxFn = bool( const elementType& element, const AABB& bbox );
		// If this is a non-point, how much of it is inside this box?
		// Returned value can be any
		using BoxOccupancyFn = float( const elementType& element, const AABB& bbox );
		// With these elements loaded, should this node subdivide any further?
		using ShouldSubdivideFn = bool( const NodeType& );

	public:
		OctreeStatic() = default;
		OctreeStatic( const OctreeStatic& octree ) = delete;
		OctreeStatic( OctreeStatic&& octree ) = default;
		OctreeStatic& operator=( OctreeStatic&& octree ) = default;

		OctreeStatic( const AABB& bbox, std::function<IntersectsBoxFn> intersectsBoxFunction, 
			std::function<BoxOccupancyFn> boxOccupancyFunction,
			std::function<ShouldSubdivideFn> shouldSubdivideFunction )
			: octreeBox( bbox ), intersectsBox( intersectsBoxFunction ),
			occupiesBox( boxOccupancyFunction ), shouldSubdivide( shouldSubdivideFunction )
		{
			Initialise( bbox, intersectsBoxFunction, boxOccupancyFunction, shouldSubdivideFunction );
		}

		void Initialise( const AABB& bbox, std::function<IntersectsBoxFn> intersectsBoxFunction,
			std::function<BoxOccupancyFn> boxOccupancyFunction,
			std::function<ShouldSubdivideFn> shouldSubdivideFunction )
		{
			octreeBox = bbox;
			intersectsBox = intersectsBoxFunction;
			occupiesBox = boxOccupancyFunction;
			shouldSubdivide = shouldSubdivideFunction;
		}

		void AddElement( const elementType& element )
		{
			elements.push_back( element );
		}

		void AddElements( const Vector<elementType>& elementList )
		{
			elements.insert( elements.end(), elementList.begin(), elementList.end() );
		}

		void SetElements( Vector<elementType>&& elementList )
		{
			elements = std::move( elementList );
		}

		// Recursively build octree nodes
		void BuildNode( NodeType* node )
		{
			// Node is a leaf, bail out
			if ( !shouldSubdivide( *node ) )
			{
				return;
			}

			// The node can be subdivided, create the child nodes
			// and figure out which element belongs to which node
			node->CreateChildren( nodes );
			node->ForEachElement( [&]( elementType* element )
				{
					// If the element is non-point and intersects with multiple
					// nodes, determine which one it'll ultimately belong to
					Vector<NodeType*> intersectingNodes;
					intersectingNodes.reserve( 8U );
					node->ForEachChild( [&]( NodeType* child )
						{
							if ( intersectsBox( *element, child->GetBoundingBox() ) )
							{
								intersectingNodes.push_back( child );
							}
						} );
					// No intersections at all, bail out
					if ( intersectingNodes.empty() )
					{
						return;
					}

					// It is only in one node, or an occupancy function
					// wasn't provided, don't bother checking spatial occupancy
					if ( intersectingNodes.size() == 1U || !occupiesBox )
					{
						intersectingNodes.front()->AddElement( element );
						return;
					}

					// Calculate surface area or volume inside each node
					NodeType* belongingNode = intersectingNodes.front();
					float maxOccupancy = -99999.0f;
					for ( auto& intersectingNode : intersectingNodes )
					{
						float occupancy = occupiesBox( *element, intersectingNode->GetBoundingBox() );
						if ( occupancy > maxOccupancy )
						{
							belongingNode = intersectingNode;
							maxOccupancy = occupancy;
						}
					}
					
					// Finally, add the thing
					belongingNode->AddElement( element );
				} );

			// Now that we've done the heavy work, go down the tree
			node->ForEachChild( [&]( NodeType* child )
				{
					BuildNode( child );
				} );
		}

		// Rebuild the tree
		void Rebuild()
		{
			// Clear the tree and put the root node in
			leaves.clear();
			nodes.clear();
			auto& node = nodes.emplace_back( octreeBox );

			// No elements, root node is empty
			if ( elements.empty() )
			{
				return;
			}

			// Fill it with all elements
			for ( auto& element : elements )
			{
				if ( intersectsBox( element, octreeBox ) )
				{
					node.AddElement( &element );
				}
			}

			// Recursively subdivide the tree
			BuildNode( &node );

			// Now that the tree is built, find all leaf nodes
			for ( auto& node : nodes )
			{
				if ( node.IsLeaf() )
				{
					leaves.push_back( &node );
				}
			}
		}

	public: // Some getters'n'stuff
		const Vector<elementType>& GetElements() const
		{
			return elements;
		}

		const AABB& GetBoundingBox() const
		{
			return octreeBox;
		}

		const LinkedList<NodeType>& GetNodes() const
		{
			return nodes;
		}

		const Vector<NodeType*>& GetLeaves() const
		{
			return leaves;
		}

	private:
		AABB octreeBox;
		std::function<IntersectsBoxFn> intersectsBox;
		std::function<BoxOccupancyFn> occupiesBox;
		std::function<ShouldSubdivideFn> shouldSubdivide;
		Vector<elementType> elements;
		LinkedList<NodeType> nodes;
		Vector<NodeType*> leaves;
	};
}

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
		
		const auto intersectsBox = []( const Vec3& element, const AABB& bbox )
		{
			return bbox.IsInside( element );
		};

		const auto occupiesBox = []( const Vec3& element, const AABB& bbox )
		{	// This is called for intersected boxes, and a point is always fully occupied by a volume
			return 1.0f;
		};

		const auto shouldSubdivide = []( const OctreeStatic<Vec3>::NodeType& node )
		{	// This is our threshold. If there's more than 50 elements inside a node,
			// it'll be subdivided
			//return node.GetNumElements() > 50;
			
			// Alternative heuristic: reverse density and distance from centre
			const int32_t& numElements = node.GetNumElements();
			
			const adm::Vec3 nodeCentre = node.GetBoundingBox().GetCentre();
			adm::Vec3 averageCentre;
			node.ForEachElement( [&]( adm::Vec3* element )
				{
					averageCentre += *element;
				} );
			averageCentre /= float( numElements );

			const float diagonal = node.GetBoundingBox().Diagonal();
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

		octree.Initialise( octreeBox, intersectsBox, occupiesBox, shouldSubdivide );

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

			renderBbox( node->GetBoundingBox(), sectorColour );
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
	adm::OctreeStatic<adm::Vec3> octree;

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
