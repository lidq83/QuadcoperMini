#include <rand.h>

static unsigned int rand_seed = 0;

void srand(unsigned int seed)
{
    rand_seed = seed;
}

unsigned int rand()
{
	rand_seed = rand_seed * 1103515245 + 12345;
	return (unsigned int) (rand_seed / 65536) % 32768;
}