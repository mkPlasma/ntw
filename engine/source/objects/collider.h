#pragma once

#include"objects/object.h"

class Portal;


struct Collider{
	Hitbox* hitbox;
	Hitbox hitboxTransformed;

	Object* parent;
	Portal* portal;

	Collider() : hitbox(nullptr), parent(nullptr), portal(nullptr) {}
};
