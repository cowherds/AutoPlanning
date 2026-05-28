"""SUPER-style finite state machine (fsm/fsm.h port)."""

from __future__ import annotations

from enum import Enum, auto


class MachineState(Enum):
    INIT = auto()
    WAIT_GOAL = auto()
    GENERATE_TRAJ = auto()
    FOLLOW_TRAJ = auto()
    EMER_STOP = auto()


STATE_LABEL = {
    MachineState.INIT: "INIT",
    MachineState.WAIT_GOAL: "WAIT_GOAL",
    MachineState.GENERATE_TRAJ: "GENERATE_TRAJ",
    MachineState.FOLLOW_TRAJ: "FOLLOW_TRAJ",
    MachineState.EMER_STOP: "EMER_STOP",
}


class SuperFSM:
    """
    Lightweight FSM aligned with SUPER/super_planner FSM transitions.

    INIT -> WAIT_GOAL -> GENERATE_TRAJ -> FOLLOW_TRAJ
    FOLLOW_TRAJ --replan fail/emer--> EMER_STOP -> WAIT_GOAL
    """

    def __init__(self) -> None:
        self.state = MachineState.INIT
        self.started = False
        self.plan_from_rest = False
        self.finish_plan = False
        self.new_goal = False

    def label(self) -> str:
        return STATE_LABEL[self.state]

    def on_system_start(self) -> None:
        self.started = True

    def on_new_goal(self) -> None:
        self.new_goal = True

    def change(self, new_state: MachineState) -> None:
        self.state = new_state

    def step_wait_goal(self) -> MachineState:
        if self.new_goal:
            self.change(MachineState.GENERATE_TRAJ)
        return self.state

    def step_generate_traj(self, plan_ok: bool, close_to_goal: bool) -> MachineState:
        if close_to_goal:
            self.new_goal = False
            self.finish_plan = True
            self.change(MachineState.WAIT_GOAL)
            return self.state
        if plan_ok:
            self.new_goal = False
            self.plan_from_rest = True
            self.finish_plan = False
            self.change(MachineState.FOLLOW_TRAJ)
        return self.state

    def step_follow_traj(self, replan_emer: bool) -> MachineState:
        if replan_emer:
            self.change(MachineState.EMER_STOP)
        return self.state

    def step_emer_stop(self) -> MachineState:
        self.change(MachineState.WAIT_GOAL)
        return self.state

    def consume_plan_from_rest(self) -> bool:
        if self.plan_from_rest:
            self.plan_from_rest = False
            return True
        return False

    def should_replan(self) -> bool:
        if self.state != MachineState.FOLLOW_TRAJ:
            return False
        if self.finish_plan:
            return False
        # Match SUPER behavior: skip one cycle right after plan-from-rest,
        # then enable continuous rolling replanning.
        if self.plan_from_rest:
            self.plan_from_rest = False
            return False
        return True
