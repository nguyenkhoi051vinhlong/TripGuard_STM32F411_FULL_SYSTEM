// Minimal freestanding declarations for the Windows host parser test only.
#pragma once
#include <stddef.h>
void *memset(void *,int,size_t);
void *memmove(void *,const void *,size_t);
int memcmp(const void *,const void *,size_t);
