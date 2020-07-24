#pragma once

#include"objects/object.h"
#include"objects/portal.h"


struct Collider{
	Hitbox* hitbox;
	Hitbox hitboxTransformed;

	Object* parent;
	Portal* portal;
};
