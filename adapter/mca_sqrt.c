// sqrtf randomly rounded for the step lane's Verificarlo build, linked with -Wl,--wrap=sqrtf so that
// upstream's source is unchanged. Verificarlo instruments + - * / but not libm, and sqrtf is the one libm
// call whose result Box3D's step rounds. B3_ORACLE_SQRT selects: "nearest" calls the real sqrtf;
// "random:<seed>" takes the correctly rounded double sqrt and rounds it to one of its two neighbouring
// floats with probability by distance, from a stream seeded per lane, keeping exact results exact.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

float __real_sqrtf(float x);
float __wrap_sqrtf(float x);

static int mode = -1;
static uint64_t state;

static void configure(void)
{
	const char* value = getenv("B3_ORACLE_SQRT");
	if (value != NULL && strcmp(value, "nearest") == 0) mode = 0;
	else if (value != NULL && strncmp(value, "random:", 7) == 0)
	{
		mode = 1;
		state = strtoull(value + 7, NULL, 10);
	}
	else
	{
		fprintf(stderr, "B3_ORACLE_SQRT must be nearest or random:<seed>\n");
		exit(2);
	}
}

// SplitMix64.
static uint64_t next(void)
{
	uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
	z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
	z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
	return z ^ (z >> 31);
}

float __wrap_sqrtf(float x)
{
	if (mode < 0) configure();
	if (mode == 0) return __real_sqrtf(x);
	double y = sqrt((double)x);
	float nearest = (float)y;
	if (!(x > 0.0f) || isinf(x) || (double)nearest == y) return nearest;
	float lo = (double)nearest < y ? nearest : nextafterf(nearest, -INFINITY);
	float hi = (double)nearest < y ? nextafterf(nearest, INFINITY) : nearest;
	double up = (y - (double)lo) / ((double)hi - (double)lo);
	return (double)(next() >> 11) * 0x1p-53 < up ? hi : lo;
}
