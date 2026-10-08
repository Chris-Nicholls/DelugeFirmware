// Heap-backed stand-ins for the Deluge's general memory allocator entry points.
#include "memory/memory_allocator_interface.h"
#include <cstdlib>

void* allocMaxSpeed(uint32_t requiredSize, void*) {
	return malloc(requiredSize);
}

void* allocLowSpeed(uint32_t requiredSize, void*) {
	return malloc(requiredSize);
}

extern "C" {
void* delugeAlloc(unsigned int requiredSize, bool) {
	return malloc(requiredSize);
}
void delugeDealloc(void* address) {
	free(address);
}
}
