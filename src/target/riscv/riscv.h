#ifndef RISCV_H
#define RISCV_H

struct riscv_program;

#include <stdint.h>
#include "opcodes.h"
#include "gdb_regs.h"
#include "jtag/jtag.h"

/* The register cache is statically allocated. */
#define RISCV_MAX_HARTS 32
#define RISCV_MAX_REGISTERS 5000
#define RISCV_MAX_TRIGGERS 32
#define RISCV_MAX_HWBPS 16

#define DEFAULT_COMMAND_TIMEOUT_SEC		2
#define DEFAULT_RESET_TIMEOUT_SEC		30

extern struct target_type riscv011_target;
extern struct target_type riscv013_target;

/*
 * Definitions shared by code supporting all RISC-V versions.
 */
typedef uint64_t riscv_reg_t;
typedef uint32_t riscv_insn_t;
typedef uint64_t riscv_addr_t;

enum riscv_halt_reason {
	RISCV_HALT_INTERRUPT,
	RISCV_HALT_BREAKPOINT,
	RISCV_HALT_SINGLESTEP,
	RISCV_HALT_TRIGGER,
	RISCV_HALT_UNKNOWN,
	RISCV_HALT_ERROR
};

typedef struct {
	struct target *target;
	unsigned custom_number;
} riscv_reg_info_t;

typedef struct {
	unsigned dtm_version;

	struct command_context *cmd_ctx;
	void *version_specific;

	/* The hart that the RTOS thinks is currently being debugged. */
	int rtos_hartid;

	/* The hart that is currently being debugged.  Note that this is
	 * different than the hartid that the RTOS is expected to use.  This
	 * one will change all the time, it's more of a global argument to
	 * every function than an actual */
	int current_hartid;

	/* Enough space to store all the registers we might need to save. */
	/* FIXME: This should probably be a bunch of register caches. */
	uint64_t saved_registers[RISCV_MAX_HARTS][RISCV_MAX_REGISTERS];
	bool valid_saved_registers[RISCV_MAX_HARTS][RISCV_MAX_REGISTERS];

	/* OpenOCD's register cache points into here. This is not per-hart because
	 * we just invalidate the entire cache when we change which hart is
	 * selected. */
	uint64_t reg_cache_values[RISCV_MAX_REGISTERS];

	/* Single buffer that contains all register names, instead of calling
	 * malloc for each register. Needs to be freed when reg_list is freed. */
	char *reg_names;

	/* It's possible that each core has a different supported ISA set. */
	int xlen[RISCV_MAX_HARTS];
	riscv_reg_t misa[RISCV_MAX_HARTS];

	/* If the target doesn't implement MISA register, use this value */
	riscv_reg_t default_misa;

	/* The number of triggers per hart. */
	unsigned trigger_count[RISCV_MAX_HARTS];

	/* For each physical trigger, contains -1 if the hwbp is available, or the
	 * unique_id of the breakpoint/watchpoint that is using it.
	 * Note that in RTOS mode the triggers are the same across all harts the
	 * target controls, while otherwise only a single hart is controlled. */
	int trigger_unique_id[RISCV_MAX_HWBPS];

	/* The number of entries in the debug buffer. */
	int debug_buffer_size[RISCV_MAX_HARTS];

	/* This avoids invalidating the register cache too often. */
	bool registers_initialized;

	/* This hart contains an implicit ebreak at the end of the program buffer. */
	bool impebreak;

	bool triggers_enumerated;

	/* Decremented every scan, and when it reaches 0 we clear the learned
	 * delays, causing them to be relearned. Used for testing. */
	int reset_delays_wait;

	/* This target has been prepped and is ready to step/resume. */
	bool prepped;
	/* This target was selected using hasel. */
	bool selected;

	/* Helper functions that target the various RISC-V debug spec
	 * implementations. */
	int (*get_register)(struct target *target,
		riscv_reg_t *value, int hid, int rid);
	int (*set_register)(struct target *, int hartid, int regid,
			uint64_t value);
	int (*select_current_hart)(struct target *);
	bool (*is_halted)(struct target *target);
	/* Resume this target, as well as every other prepped target that can be
	 * resumed near-simultaneously. Clear the prepped flag on any target that
	 * was resumed. */
	int (*resume_go)(struct target *target);
	int (*step_current_hart)(struct target *target);
	int (*on_halt)(struct target *target);
	/* Get this target as ready as possible to resume, without actually
	 * resuming. */
	int (*resume_prep)(struct target *target);
	int (*halt_prep)(struct target *target);
	int (*halt_go)(struct target *target);
	int (*on_step)(struct target *target);
	enum riscv_halt_reason (*halt_reason)(struct target *target);
	int (*write_debug_buffer)(struct target *target, unsigned index,
			riscv_insn_t d);
	riscv_insn_t (*read_debug_buffer)(struct target *target, unsigned index);
	int (*execute_debug_buffer)(struct target *target);
	int (*dmi_write_u64_bits)(struct target *target);
	void (*fill_dmi_write_u64)(struct target *target, char *buf, int a, uint64_t d);
	void (*fill_dmi_read_u64)(struct target *target, char *buf, int a);
	void (*fill_dmi_nop_u64)(struct target *target, char *buf);

	int (*authdata_read)(struct target *target, uint32_t *value);
	int (*authdata_write)(struct target *target, uint32_t value);

	int (*dmi_read)(struct target *target, uint32_t *value, uint32_t address);
	int (*dmi_write)(struct target *target, uint32_t address, uint32_t value);

	int (*test_sba_config_reg)(struct target *target, target_addr_t legal_address,
			uint32_t num_words, target_addr_t illegal_address, bool run_sbbusyerror_test);

	int (*test_compliance)(struct target *target);

	/* How many harts are attached to the DM that this target is attached to? */
	int (*hart_count)(struct target *target);
	unsigned (*data_bits)(struct target *target);
} riscv_info_t;

typedef struct {
	uint8_t tunneled_dr_width;
	struct scan_field tunneled_dr[4];
} riscv_bscan_tunneled_scan_context_t;


/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
extern int riscv_command_timeout_sec;

/* Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
extern int riscv_reset_timeout_sec;

extern bool riscv_prefer_sba;

extern bool riscv_enable_virtual;

/* Everything needs the RISC-V specific info structure, so here's a nice macro
 * that provides that. */
static inline riscv_info_t *riscv_info(const struct target *target) __attribute__((unused));
static inline riscv_info_t *riscv_info(const struct target *target)
{ return target->arch_info; }
#define RISCV_INFO(R) riscv_info_t *R = riscv_info(target);

extern uint8_t ir_dtmcontrol[4];
extern struct scan_field select_dtmcontrol;
extern uint8_t ir_dbus[4];
extern struct scan_field select_dbus;
extern uint8_t ir_idcode[4];
extern struct scan_field select_idcode;

extern struct scan_field select_user4;
extern struct scan_field *bscan_tunneled_select_dmi;
extern uint32_t bscan_tunneled_select_dmi_num_fields;
extern uint8_t bscan_zero[4];
extern uint8_t bscan_one[4];
typedef enum { BSCAN_TUNNEL_NESTED_TAP, BSCAN_TUNNEL_DATA_REGISTER } bscan_tunnel_type_t;
extern int bscan_tunnel_ir_width;
extern bscan_tunnel_type_t bscan_tunnel_type;

uint32_t dtmcontrol_scan_via_bscan(struct target *target, uint32_t out);
void select_dmi_via_bscan(struct target *target);

/*** OpenOCD Interface */
int riscv_openocd_poll(struct target *target);

int riscv_halt(struct target *target);

int riscv_resume(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution
);

int riscv_openocd_step(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints
);

int riscv_openocd_assert_reset(struct target *target);
int riscv_openocd_deassert_reset(struct target *target);

/*** RISC-V Interface ***/

/* Initializes the shared RISC-V structure. */
void riscv_info_init(struct target *target, riscv_info_t *r);
int riscv_init_target_info(struct command_context *cmd_ctx,
		struct target *target, riscv_info_t *info);

/* Steps the hart that's currently selected in the RTOS, or if there is no RTOS
 * then the only hart. */
int riscv_step_rtos_hart(struct target *target);

bool riscv_supports_extension(struct target *target, int hartid, char letter);

/* Returns XLEN for the given (or current) hart. */
unsigned riscv_xlen(const struct target *target);
int riscv_xlen_of_hart(const struct target *target, int hartid);

bool riscv_rtos_enabled(const struct target *target);

/* Sets the current hart, which is the hart that will actually be used when
 * issuing debug commands. */
int riscv_set_current_hartid(struct target *target, int hartid);
int riscv_current_hartid(const struct target *target);

/*** Support functions for the RISC-V 'RTOS', which provides multihart support
 * without requiring multiple targets.  */

/* When using the RTOS to debug, this selects the hart that is currently being
 * debugged.  This doesn't propogate to the hardware. */
void riscv_set_all_rtos_harts(struct target *target);
void riscv_set_rtos_hartid(struct target *target, int hartid);

/* Lists the number of harts in the system, which are assumed to be
 * concecutive and start with mhartid=0. */
int riscv_count_harts(struct target *target);

/* Returns TRUE if the target has the given register on the given hart.  */
bool riscv_has_register(struct target *target, int hartid, int regid);

/* Returns the value of the given register on the given hart.  32-bit registers
 * are zero extended to 64 bits.  */
int riscv_set_register(struct target *target, enum gdb_regno i, riscv_reg_t v);
int riscv_set_register_on_hart(struct target *target, int hid, enum gdb_regno rid, uint64_t v);
int riscv_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno r);
int riscv_get_register_on_hart(struct target *target, riscv_reg_t *value,
		int hartid, enum gdb_regno regid);

/* Checks the state of the current hart -- "is_halted" checks the actual
 * on-device register. */
bool riscv_is_halted(struct target *target);
enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid);

/* These helper functions let the generic program interface get target-specific
 * information. */
size_t riscv_debug_buffer_size(struct target *target);

riscv_insn_t riscv_read_debug_buffer(struct target *target, int index);
int riscv_write_debug_buffer(struct target *target, int index, riscv_insn_t insn);
int riscv_execute_debug_buffer(struct target *target);

void riscv_fill_dmi_nop_u64(struct target *target, char *buf);
void riscv_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
void riscv_fill_dmi_read_u64(struct target *target, char *buf, int a);
int riscv_dmi_write_u64_bits(struct target *target);

/* Invalidates the register cache. */
void riscv_invalidate_register_cache(struct target *target);

/* Returns TRUE when a hart is enabled in this target. */
bool riscv_hart_enabled(struct target *target, int hartid);

int riscv_enumerate_triggers(struct target *target);

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int riscv_remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint);
int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint);
int riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_wp_address);

int riscv_init_registers(struct target *target);

void riscv_semihosting_init(struct target *target);
int riscv_semihosting(struct target *target, int *retval);

void riscv_add_bscan_tunneled_scan(struct target *target, struct scan_field *field,
		riscv_bscan_tunneled_scan_context_t *ctxt);

#endif
