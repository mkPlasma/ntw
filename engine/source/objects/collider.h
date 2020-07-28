#pragma once

#include"objects/object.h"

class Portal;


struct Collider{
	Hitbox* hitbox;
	Hitbox hitboxTransformed;

	Object* parent;
	Portal* portal;
};
