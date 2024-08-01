class Double IntegratorInterfacefinal: public RobotInterface

```cpp
class Double IntegratorInterfacefinal: public RobotInterface
//reference, optimal control problem, solver initializer

class MPC_ROS_Interface
//some node for MPC policy publisher and observation subscriber

class RosReferenceManager final : public ReferenceManagerDecorator

```



advance -- Continue the program up to the given location (same form as args for break command).
attach -- Attach to a process or file outside of GDB.
continue -- Continue program being debugged, after signal or breakpoint.
detach -- Detach a process or file previously attached.
detach checkpoint -- Detach from a checkpoint (experimental).
detach inferiors -- Detach from inferior ID (or list of IDS).
disconnect -- Disconnect from a target.
finish -- Execute until selected stack frame returns.
handle -- Specify how to handle signals.
inferior -- Use this command to switch between inferiors.
interrupt -- Interrupt the execution of the debugged program.
jump -- Continue program being debugged at specified line or address.
kill -- Kill execution of program being debugged.
kill inferiors -- Kill inferior ID (or list of IDs).
next -- Step program, proceeding through subroutine calls.
nexti -- Step one instruction, but proceed through subroutine calls.
queue-signal -- Queue a signal to be delivered to the current thread when it is resumed.
--Type `<RET>` for more, q to quit, c to continue without paging--
reverse-continue -- Continue program being debugged but run it in reverse.
reverse-finish -- Execute backward until just before selected stack frame is called.
reverse-next -- Step program backward, proceeding through subroutine calls.
reverse-nexti -- Step backward one instruction, but proceed through called subroutines.
reverse-step -- Step program backward until it reaches the beginning of another source line.
reverse-stepi -- Step backward exactly one instruction.
run -- Start debugged program.
signal -- Continue program with the specified signal.
start -- Start the debugged program stopping at the beginning of the main procedure.
starti -- Start the debugged program stopping at the first instruction.
step -- Step program until it reaches a different source line.
stepi -- Step one instruction exactly.
taas -- Apply a command to all threads (ignoring errors and empty output).
target -- Connect to a target machine or process.
target core -- Use a core file as a target.
target ctf -- (Use a CTF directory as a target.
target exec -- Use an executable file as a target.
target extended-remote -- Use a remote computer via a serial line, using a gdb-specific protocol.
target native -- Native process (started by the "run" command).
target record-btrace -- Collect control-flow trace and provide the execution history.
target record-core -- Log program while executing and replay execution from log.
target record-full -- Log program while executing and replay execution from log.
target remote -- Use a remote computer via a serial line, using a gdb-specific protocol.
target tfile -- Use a trace file as a target.
task -- Use this command to switch between Ada tasks.
tfaas -- Apply a command to all frames of all threads (ignoring errors and empty output).
thread -- Use this command to switch between threads.
thread apply -- Apply a command to a list of threads.
thread apply all -- Apply a command to all threads.
thread find -- Find threads that match a regular expression.
thread name -- Set the current thread's name.
until -- Execute until past the current line or past a LOCATION.


**DoubleIntegratorMpcNode.cpp**

6 thread

thrad 1:
	ocs2::MPC_ROS_Interface::launchNodes
		ocs2::MPC_ROS_Interface::spin for NodeHandle::subscribe()/NodeHandle::advertise()/etc


thread 6:
	ocs2::MPC_ROS_Interface::publisherWorker (MPC_ROS_Interface.cpp)

```cpp

```
