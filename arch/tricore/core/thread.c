#include <kernel.h>
#include <ksched.h>
#include <offsets_short.h>
#include <wait_q.h>
#include <thread.h>

#if 0
void arch_switch(void *switch_to, void **switched_from)
{
/*
	struct tricore_context *ctx = *switched_from;

	SVLCX();
	ctx->pcxi = __mfcr(EE_CPU_REG_PSW);
	*switched_from = ctx;

	if (switch_to->pcxi == NULL) {
		switch_to->pcxi = csa;
	}

	EE_CPU_REG_PSW = switch_to->pcxi;
	__mfcr(EE_CPU_REG_PSW);
	RSLCX();
*/

	struct tricore_context *from = *switched_from;
	struct tricore_context *to = switched_to;

	STLCX(from->lower);
	STUCX(from->upper);

	LDLCX(to->lower);
	LDUCX(to->upper);
}
#endif

void ticore_csa_init(struct tricore_context *context)
{
	struct lower_ctx *nctx, *ctx = context->csa;

	ctx = (ctx + 1) & ~sizeof(struct lower_ctx);

	context->csa_head = ctx >> 28 | ((ctx >> 6) & 0xffff);

	nctx = ctx;
	for (i = 0; i < CONFIG_TRICORE_CALL_DEPTH; i++, ctx++) {
		++nctx;
		ctx->pcxi = nctx >> 28 | ((nctx >> 6) & 0xffff);
	}

	ctx->pcxi = 0;
	context->csa_tail = ctx >> 28 | ((ctx >> 6) & 0xffff);
}

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack,
		     size_t stackSize, k_thread_entry_t pEntry,
		     void *parameter1, void *parameter2, void *parameter3,
		     int priority, unsigned int options)
{
	char *pStackMem = Z_THREAD_STACK_BUFFER(stack);
	char *stackEnd;
	struct tricore_context *ctx;

	pStackMem += STACK_GUARD_SIZE;
	stackEnd = pStackMem + stackSize;

	z_new_thread_init(thread, pStackMem, stackSize);

	ctx = (struct tricore_context *)(
			Z_STACK_PTR_ALIGN(stackEnd) -
			sizeof(struct tricore_context));

	ctx->csa = pStackMem + stackSize + ARCH_TRICORE_CSA_SIZE;
	ticore_csa_init(ctx);

	ctx->upper.psw = 0xB80 | (CONFIG_TRICORE_CALL_DEPTH & GENMASK(6, 0));
	ctx->upper.a10 = stackEnd;
	ctx->lower.pcxi = 0;
	ctx->upper.pcxi = 0;
	ctx->upper.a11 = ((u32_t)z_thread_entry);
	ctx->lower.a4 = (u32_t)pEntry;
	ctx->lower.a5 = (u32_t)parameter1;
	ctx->lower.a6 = (u32_t)parameter2;
	ctx->lower.a7 = (u32_t)parameter3;

	thread->switch_handle = ctx;
}
