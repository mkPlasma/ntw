#include "error.h"

// Throw a fatal error and terminate process
void ntw::fatalError(const string& msg){
	fprintf(stderr, "FATAL ERROR: %s\n", msg.c_str());
	system("pause");
	exit(EXIT_FAILURE);
}

// Print error message
void ntw::error(const string& msg){
	fprintf(stderr, "ERROR: %s\n", msg.c_str());
}

// Print warning message
void ntw::warning(const string& msg){
	fprintf(stderr, "WARNING: %s\n", msg.c_str());
}
