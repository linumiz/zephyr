#ifndef ZEPHYR_INCLUDE_ARCH_TRICORE_THREAD_H_

struct lower_ctx {
	int pcxi;
	int a11;
	int a2;
	int a3;
	int d0;
	int d1;
	int d2;
	int d3;
	int a4;
	int a5;
	int a6;
	int a7;
	int d4;
	int d5;
	int d6;
	int d7;
};

struct upper_ctx {
	int pcxi;
	int psw;
	int a10;
	int a11;
	int d8;
	int d9;
	int d10;
	int d11;
	int a12;
	int a13;
	int a14;
	int a15;
	int d12;
	int d13;
	int d14;
	int d15;
};

struct tricore_context {
	struct lower_ctx lower;
	struct upper_ctx upper;
	unsigned int csa_head;
	unsigned int csa_tail;
	void *csa;
};

typedef struct tricore_context _tricore_context_t

#endif
