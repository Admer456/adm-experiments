
#pragma once

#include "debug_draw.hpp"

struct UserCommand
{
	UserCommand() = default;
	UserCommand( const UserCommand& uc ) = default;
	UserCommand& operator=( const UserCommand& uc ) = default;

	enum Actions
	{
		Action1 = 1 << 0,
		Action2 = 1 << 1,
		Reload = 1 << 2,
		Speed = 1 << 3,
		Crouch = 1 << 4,
		Jump = 1 << 5,
	};

	float forward{};
	float right{};
	float up{};

	int flags{};

	// Relative mouse coords
	float mouseX{};
	float mouseY{};

	// Absolute mouse coords
	float mouseWindowX{};
	float mouseWindowY{};
};

class IApplication
{
public:
	virtual bool Init() = 0;
	virtual void Shutdown() = 0;

	virtual void Update( const float& deltaTime, const float& time, const UserCommand& uc ) = 0;

	virtual const float* GetViewProjectionMatrix() const = 0;
};

struct ApplicationInstance
{
	const char* name{};
	IApplication* app{};
};

#define DeclareExperiment( applicationClass ) \
ApplicationInstance GetApplication() \
{ \
	return { #applicationClass, new applicationClass() };\
}
