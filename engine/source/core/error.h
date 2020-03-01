#pragma once

#include<stdlib.h>
#include<stdio.h>
#include<string>


namespace ntw{

	using std::string;

	// Print fatal error message and terminate process
	void fatalError(const string& msg);

	// Print error message
	void error(const string& msg);

	// Print warning message
	void warning(const string& msg);
}
