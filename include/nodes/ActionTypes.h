#pragma once

struct WorldState;
class MotionController;

typedef void (*ACTION_EXEC_FUNC)(const WorldState&, MotionController*);
