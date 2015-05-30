#include <stdio.h>
#include <stdint.h>

uint32_t x;
extern uint32_t *y;
extern uint32_t z[];

int main(void)
{
	printf("'int x'   is a global located at program area (&x): 0x%lx\n", &x);
	printf("'int *y'  is a global located at program area (&y): 0x%lx\n", &y);
	printf("'int z[]' is a global located at program area (&z): 0x%lx\n\n", &z);

	printf("x = 0x%x\n", x);
	printf("y = 0x%x\n", y);
	printf("z = 0x%x\n\n", z);

	printf("*x = <not-applicable>\n");
	printf("*y = <segmentation fault> since 0x%x is not mapped\n", y);
	printf("*z = 0x%x\n\n", *z);

	printf("x[0] = <not-applicable>\n");
	printf("y[0] = <segmentation fault> since 0x%x is not mapped\n", y);
	printf("z[0] = 0x%x\n", z[0]);
}
