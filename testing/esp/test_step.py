import logging
import unittest
import debug_backend as dbg
from time import sleep
from debug_backend_tests import *


def get_logger():
    """ Returns logger for this module
    """
    return logging.getLogger(__name__)


########################################################################
#                         TESTS IMPLEMENTATION                         #
########################################################################

class StepTestsImpl():
    """ Stepping test cases generic for dual and single core modes
    """

    def do_step_over_bp_check(self, funcs):
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
        old_pc = self.gdb.get_reg('pc')
        faddr = self.gdb.extract_exec_addr(self.gdb.data_eval_expr('&%s' % funcs[0]))
        self.assertEqual(old_pc, faddr)
        self.step(insn=True) # step over movi
        new_pc = self.gdb.get_reg('pc')
        self.assertTrue(((new_pc - old_pc) == 2) or ((new_pc - old_pc) == 3))
        old_pc = new_pc
        self.step(insn=True, stop_rsn=dbg.TARGET_STOP_REASON_BP) # step over nop
        new_pc = self.gdb.get_reg('pc')
        faddr = self.gdb.extract_exec_addr(self.gdb.data_eval_expr('&%s' % funcs[1]))
        self.assertEqual(new_pc, faddr)
        self.assertTrue(((new_pc - old_pc) == 2) or ((new_pc - old_pc) == 3))

    def test_step_over_bp(self):
        """
            This test checks that debugger can step over breakpoint.
            1) Select appropriate sub-test number on target.
            2) Set several breakpoints to cover all types of them (HW, SW). Two BPs of every type.
            3) Resume target and wait for the first breakpoint to hit.
            4) Check that target has stopped in the right place.
            5) Performs step from stop point (to the second breakpoint of that type).
            6) Check that PC changed correctly.
            7) Repeat steps 3-6 several times for every type of breakpoints.
        """
        bps = ['_step_over_bp_break1', '_step_over_bp_break2',  # HW BPs
            '_step_over_bp_break3', '_step_over_bp_break4',  # SW flash BPs
            '_step_over_bp_break5', '_step_over_bp_break6']  # SW RAM BPs
        for f in bps:
            self.add_bp(f)
        self.select_sub_test(103)
        for i in range(2):
            # step from and over HW BPs
            self.do_step_over_bp_check(['_step_over_bp_break1', '_step_over_bp_break2'])
            # step from and over SW flash BPs
            self.do_step_over_bp_check(['_step_over_bp_break3', '_step_over_bp_break4'])
            # step from and over SW RAM BPs
            self.do_step_over_bp_check(['_step_over_bp_break5', '_step_over_bp_break6'])

    def do_step_over_wp_check(self, func):
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_SIGTRAP)
        cur_frame = self.gdb.get_current_frame()
        self.assertEqual(cur_frame['func'], func)
        old_pc = self.gdb.get_reg('pc')
        self.step(insn=True)
        new_pc = self.gdb.get_reg('pc')
        self.assertTrue(((new_pc - old_pc) == 2) or ((new_pc - old_pc) == 3))

    def test_step_over_wp(self):
        """
            This test checks that debugger can step over triggered watchpoint.
            1) Select appropriate sub-test number on target.
            2) Set access watchpoint.
            3) Resume target and wait for watchpoint to hit.
            4) Check that target has stopped in the right place.
            5) Performs step from stop point.
            6) Check that PC changed correctly.
            7) Repeat steps 3-6 several times.
        """
        self.wps = {'s_count1': None}
        for e in self.wps:
            self.add_wp(e, 'rw')
        self.select_sub_test(100)
        for i in range(2):
            # 'count' read
            self.do_step_over_wp_check('blink_task')
            # 'count' read
            self.do_step_over_wp_check('blink_task')
            # 'count' write
            self.do_step_over_wp_check('blink_task')

    def test_step_window_exception(self):
        # start the test, stopping at the window_exception_test function
        self.select_sub_test(200)
        bp = self.gdb.add_bp('_recursive_func')
        self.resume_exec()
        rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
        self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
        self.gdb.delete_bp(bp)

        # do "step in", 3 steps per recursion level
        for i in range(0, 57):
            get_logger().info('Step in {}'.format(i))
            self.step_in()

        # check that we have reached the end of recursion
        self.assertEqual(int(self.gdb.data_eval_expr('levels')), 1)

        # do "step out" once per recursion level
        for i in range(0, 20):
            get_logger().info('Step out {}'.format(i))
            self.step_out()

        cur_frame = self.gdb.get_current_frame()
        self.assertEqual(cur_frame['func'], 'window_exception_test')

    def test_step_over_insn_using_scratch_reg(self):
        """
            This test checks that scratch register (A3) used by OpenOCD for its internal purposes
            (e.g. target memory reads) is not corrupted by stepping over instructions which use that reg as operand.
            1) Select appropriate sub-test number on target.
            2) Set breakpoint just before test instructions.
            3) Resume target and wait for breakpoint hit.
            4) Check that target stopped at the correct location.
            5) Step over instruction which write counter value to A3.
            6) Check that A3 has correct value.
            7) Step over instruction which moves A3 to A4.
            8) Check that A4 has correct value.
            9) Increment counter.
            10) Repeat steps 3-9 several times.
        """
        self.select_sub_test(102)
        val = 100
        self.add_bp('_scratch_reg_using_task_break')
        for i in range(5):
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'scratch_reg_using_task')
            self.step(insn=True)
            reg_val = self.gdb.get_reg('a3')
            self.assertEqual(reg_val, val)
            self.step(insn=True)
            reg_val = self.gdb.get_reg('a4')
            self.assertEqual(reg_val, val)
            val += 1

    def test_step_multimode(self):
        """
        1) Step over lines multiple times. Checks: correct line number change.
        2) Step over instructions multiple times. Checks: correct insn addr change.
        3) Combine stepping over lines and instructions.
        4) Resume target (continue).
        5) Interrupt target (ctrl+c).
        6) Repeat steps 1-5 several times.
        """
        self.select_sub_test(104)
        self.add_bp("fib_while")

        for i in range(3):
            get_logger().info('loop ' + str(i))

            # catching of bp
            self.resume_exec()
            get_logger().info('bp1 ')
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], "fibonacci_calc")

            for s in range(3):
                get_logger().info('line test ' + str(s))
                line_a = (self.gdb.get_current_frame())['line']
                self.step()
                line_b = (self.gdb.get_current_frame())['line']
                get_logger().info('line_a' + str(line_a))
                get_logger().info('line_b' + str(line_b))
                line_dif = int(line_b)-int(line_a)
                self.assertTrue(line_dif == 1)
                get_logger().info('done')

            for p in range(3):
                get_logger().info('pc test ' + str(p))
                pc_a = self.gdb.get_reg('pc')
                self.step(insn=True)
                pc_b = self.gdb.get_reg('pc')
                get_logger().info('pc_a' + str(pc_a))
                get_logger().info('pc_b' + str(pc_b))
                pc_dif = int(pc_b)-int(pc_a)
                get_logger().info('pc_dif' + str(pc_dif))
                self.assertTrue((pc_dif == 2) or (pc_dif == 3))
                get_logger().info('done')

            # catching of bp
            self.resume_exec()
            get_logger().info('bp2 ')
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], "fibonacci_calc")

            for m in range(3):
                get_logger().info('mixed test ' + str(m))
                line_a = (self.gdb.get_current_frame())['line']
                self.step()
                pc_a = self.gdb.get_reg('pc')
                self.step(insn=True)
                pc_b = self.gdb.get_reg('pc')
                get_logger().info('pc_a' + str(pc_a))
                get_logger().info('pc_b' + str(pc_b))
                line_b = (self.gdb.get_current_frame())['line']
                get_logger().info('line_a' + str(line_a))
                get_logger().info('line_b' + str(line_b))
                line_dif = int(line_b)-int(line_a)
                pc_dif = int(pc_b)-int(pc_a)
                self.assertTrue(line_dif == 1)
                self.assertTrue((pc_dif == 2) or (pc_dif == 3))
                get_logger().info("done")

            get_logger().info("interrupt test")
            self.clear_bps()
            self.resume_exec()
            sleep(1) #  let it works some time
            self.interrupt()
            get_logger().info("done")
            self.add_bp("fib_while") #  restore bp

    def test_step_out_of_function(self):
        """
            1) Set BP inside a deep nested function
            2) Catch it
            3) Do step out until getting to the main test function
            4) Repeat 2-4 steps

        """
        self.select_sub_test(201)
        self.add_bp('nested_bottom')
        for i in range(3):

            # catching the BP
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'nested_bottom')

            # stepping out:
            self.step_out()
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'nested_middle')
            self.step_out()
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'nested_top')
            self.step_out()
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'step_out_of_function_test')

    def test_step_level5_int(self):
        """
            1) Set a breakpoint inside a level 5 interrupt vector
            2) Wait until it hits
            3) Step into the handler
            4) Return from the interrupt
        """
        self.select_sub_test(202)
        self.add_bp('_Level5Vector')
        for _ in range(3):
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            self.assertEqual(self.gdb.get_current_frame()['func'], '_Level5Vector')

            # Step into the interrupt handler
            self.step(insn=True)
            self.step(insn=True)
            self.assertEqual(self.gdb.get_current_frame()['func'], 'xt_highint5')

            # Step out of the interrupt handler
            for _ in range(12):
                self.step(insn=True)
            self.assertNotEqual(self.gdb.get_current_frame()['func'], 'xt_highint5')

    def isr_masking(self, on=True):
        if on:
            self.gdb.monitor_run("xtensa maskisr on", 5)
        else:
            self.gdb.monitor_run("xtensa maskisr off", 5)

    def get_isr_masking(self):
        _, s = self.gdb.monitor_run("xtensa maskisr", 5, output_type='stdout')
        return s.strip('\\n\\n').split("mode: ", 1)[1]

    def test_step_over_intlevel_disabled_isr(self):
        """
            This test checks ps.intlevel value after step instruction while ISRs are masked
            1) Select appropriate sub-test number on target.
            2) Set breakpoint in step_over_inst_changing_intlevel function to read write ps.intlevel
            3) Resume target and wait for brekpoint to hit.
            4) Check that target has stopped in the right place.
            5) Disable ISRs
            6) Step over instruction which changes ps value
            7) Enable ISRs
            8) Check PS and PC has correct value
            9) Repeat steps 3 several times
        """
        self.select_sub_test(120)
        self.add_bp('_step_over_intlevel_ch')
        for i in range(3):
            get_logger().info('test_step_over_intlevel_disabled_isr loop ' + str(i))
            self.resume_exec()
            rsn = self.gdb.wait_target_state(dbg.TARGET_STATE_STOPPED, 5)
            self.assertEqual(rsn, dbg.TARGET_STOP_REASON_BP)
            cur_frame = self.gdb.get_current_frame()
            self.assertEqual(cur_frame['func'], 'step_over_inst_changing_intlevel')
            old_pc = self.gdb.get_reg('pc')
            old_ps = self.gdb.get_reg('ps')
            old_masking = self.get_isr_masking();
            self.isr_masking(on=True)
            self.step(insn=True)
            self.isr_masking(on=(old_masking == 'ON'))
            new_ps = self.gdb.get_reg('ps')
            new_pc = self.gdb.get_reg('pc')
            self.assertTrue(((new_pc - old_pc) == 2) or ((new_pc - old_pc) == 3))
            get_logger().info('PS_old 0x%X', old_ps)
            get_logger().info('PS_new 0x%X', new_ps)
            self.assertEqual(old_ps & 0xF, new_ps & 0xF)

########################################################################
#              TESTS DEFINITION WITH SPECIAL TESTS                     #
########################################################################

class DebuggerStepTestsDual(DebuggerGenericTestAppTestsDual, StepTestsImpl):
    """ Test cases for dual core mode
    """
    pass

class DebuggerStepTestsDualEncrypted(DebuggerGenericTestAppTestsDualEncrypted, StepTestsImpl):
    """ Test cases for encrypted dual core mode
    """
    pass

class DebuggerStepTestsSingle(DebuggerGenericTestAppTestsSingle, StepTestsImpl):
    """ Test cases for single core mode
    """
    pass

class DebuggerStepTestsSingleEncrypted(DebuggerGenericTestAppTestsSingleEncrypted, StepTestsImpl):
    """ Test cases for encrypted single core mode
    """
    pass


