
#include <SDL.h>
#include <Precompiled.hpp>
#include <chrono>
#include <thread>

#define DEBUG_DRAW_IMPLEMENTATION
#include "IApplication.hpp"

#include <GL/glew.h>
#include "DebugDrawBackend.hpp"

using namespace std::chrono;

// Defined in the Main.cpp of each experiment
extern ApplicationInstance GetApplication();

UserCommand GenerateUserCommands( SDL_Window* window )
{
	UserCommand uc;

	const auto* states = SDL_GetKeyboardState( nullptr );
	if ( states[SDL_SCANCODE_W] )
	{
		uc.forward += 1.0f;
	}
	if ( states[SDL_SCANCODE_S] )
	{
		uc.forward -= 1.0f;
	}
	if ( states[SDL_SCANCODE_A] )
	{
		uc.right -= 1.0f;
	}
	if ( states[SDL_SCANCODE_D] )
	{
		uc.right += 1.0f;
	}
	if ( states[SDL_SCANCODE_LCTRL] )
	{
		uc.flags |= UserCommand::Crouch;
	}
	if ( states[SDL_SCANCODE_SPACE] )
	{
		uc.flags |= UserCommand::Jump;
	}
	if ( states[SDL_SCANCODE_LSHIFT] )
	{
		uc.flags |= UserCommand::Speed;
	}
	if ( states[SDL_SCANCODE_R] )
	{
		uc.flags |= UserCommand::Reload;
	}

	int x, y;
	SDL_GetRelativeMouseState( &x, &y );
	uc.mouseX = x;
	uc.mouseY = y;

	int mouseState = SDL_GetMouseState( &x, &y );
	uc.mouseWindowX = x;
	uc.mouseWindowY = y;

	if ( mouseState & SDL_BUTTON_LMASK )
	{
		uc.flags |= UserCommand::Action1;
	}
	if ( mouseState & SDL_BUTTON_RMASK )
	{
		uc.flags |= UserCommand::Action2;
	}

	return uc;
}

bool RunFrame( SDL_Window* window, IApplication* app )
{
	static float time = 0.0f;
	static float deltaTime = 0.0f;
	adm::Timer timer;

	{
		SDL_Event e;
		while ( SDL_PollEvent( &e ) )
		{
			if ( e.type == SDL_QUIT )
			{
				return false;
			}
		}
	}

	const UserCommand uc = GenerateUserCommands( window );

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	app->Update( deltaTime, time, uc );

	dd::flush();

	SDL_GL_SwapWindow( window );

	float actualDeltaTime = timer.GetElapsed( adm::Timer::Seconds );
	// Subtract one extra millisecond otherwise it'll be capped to 55fps for some reason
	float timeTil60Hz = (1.0f / 60.0f) - actualDeltaTime - 0.001f;
	if ( timeTil60Hz > 0.0f )
	{
		int us = timeTil60Hz * 1000.0f * 1000.0f;
		std::this_thread::sleep_for( microseconds( us ) );
	}

	deltaTime = timer.GetElapsed( adm::Timer::Seconds );
	time += deltaTime;

	return true;
}

int main( int argc, char** argv )
{
	SDL_Init( SDL_INIT_VIDEO | SDL_INIT_EVENTS );
	ApplicationInstance instance = GetApplication();

	SDL_Window* window = SDL_CreateWindow( instance.name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		1600, 900, SDL_WINDOW_OPENGL );

	SDL_GL_SetSwapInterval( 0 );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MAJOR_VERSION, 3 );
	SDL_GL_SetAttribute( SDL_GL_CONTEXT_MINOR_VERSION, 3 );
	SDL_GLContext context = SDL_GL_CreateContext( window );

	glewInit();

	DDRenderInterfaceCoreGL* renderBackend = new DDRenderInterfaceCoreGL();

	instance.app->Init();

	renderBackend->mvpMatrix = instance.app->GetViewProjectionMatrix();
	
	dd::initialize( renderBackend );
	
	while ( RunFrame( window, instance.app ) );
	
	dd::shutdown();
	
	instance.app->Shutdown();

	delete renderBackend;

	SDL_GL_DeleteContext( context );
	SDL_DestroyWindow( window );

	delete instance.app;
	SDL_Quit();

	return 0;
}
