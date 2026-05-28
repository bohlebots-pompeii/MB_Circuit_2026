#pragma once
#include "util/Vector2.hpp"

struct ActionHalfCircleGuard;
struct WorldState;
class MotionController;

typedef void (*ACTION_EXEC_FUNC)(const WorldState&, MotionController*);
typedef void (*ACTION_EXEC_FUNC_HCG)(const WorldState&, MotionController*, const ActionHalfCircleGuard&);
typedef bool (*ACTION_CHECK_FUNC)(const WorldState&);
typedef Vector2 (*ACTION_GET_VEC_WS)(const WorldState&);
typedef Vector2 (*ACTION_GET_VEC_WS_TARGET)(const WorldState&, const Vector2& target);