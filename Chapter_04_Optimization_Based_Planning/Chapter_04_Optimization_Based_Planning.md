**Volume 15. Motion Planning and Navigation**


# Chapter 04. Optimization Based Planning

##  

## 04.01. Trajectory Optimization Theory NLP SQP Interior Point

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

Trajectory optimization formulates robot motion planning as a continuous optimization problem in which the decision variables describe a trajectory over time. Instead of exploring a graph or randomly sampling configurations, the planner directly modifies states, controls, or trajectory parameters to minimize an objective while satisfying motion, collision, and system constraints. This approach naturally integrates path quality and dynamic feasibility.

A trajectory may be represented by discrete robot states \\(x_0,x_1,\\ldots,x_N\\), control inputs \\(u_0,u_1,\\ldots,u_{N-1}\\), or parameters of continuous curves such as splines. The optimization variables can therefore include position, orientation, velocity, acceleration, steering angle, joint configuration, or time intervals. The representation determines both the numerical complexity of the problem and the kinds of physical constraints that can be expressed.

The general trajectory optimization problem minimizes an objective \\(J(x,u)\\) subject to equality and inequality constraints. Equality constraints typically describe initial and terminal conditions, robot dynamics, or kinematic relationships, while inequality constraints impose collision avoidance, actuator limits, velocity bounds, acceleration limits, and safety margins. Consequently, practical robot planning often becomes a constrained nonlinear programming problem, commonly abbreviated as NLP.

The objective function converts desirable motion characteristics into numerical costs. Typical terms penalize trajectory length, travel time, energy consumption, control effort, acceleration, jerk, proximity to obstacles, or deviation from a reference path. Multiple terms are usually combined through weighting coefficients. Selecting these weights is important because the optimizer mathematically balances competing objectives rather than treating every desirable property as an absolute requirement.

Nonlinearity arises naturally in robotics. Robot dynamics contain trigonometric relationships and products between states and controls, while collision constraints depend nonlinearly on robot geometry and obstacle boundaries. A mobile robot may additionally have nonholonomic constraints that prohibit arbitrary lateral motion. Manipulators, legged robots, and UAVs introduce increasingly complex dynamics, making general nonlinear optimization an important foundation for optimization-based motion planning.

Trajectory optimization requires converting continuous-time motion equations into a finite-dimensional problem that a numerical solver can process. Direct transcription discretizes the trajectory at selected time points and introduces states and controls as optimization variables. Dynamics are then enforced between neighboring points through numerical integration or collocation constraints. Finer discretization improves trajectory representation but increases the number of variables and computational cost.

Direct shooting uses control inputs as primary optimization variables and obtains states by integrating the robot dynamics forward in time. This reduces explicit state variables but can produce poor numerical conditioning for long horizons because early control changes propagate through the entire trajectory. Multiple shooting divides the horizon into shorter segments and introduces intermediate states, improving numerical robustness while adding continuity constraints between segments.

Sequential Quadratic Programming, or SQP, solves a nonlinear constrained problem through a sequence of quadratic programming subproblems. At each iteration, the nonlinear objective is approximated locally, while constraints are linearized around the current trajectory. The quadratic subproblem computes a search direction, after which the trajectory is updated and the process repeats. SQP can achieve rapid local convergence when derivatives and an appropriate initial trajectory are available.

The effectiveness of SQP depends strongly on gradient information and the approximation of second-order curvature. Analytical derivatives can provide high accuracy but become difficult to maintain for complex robotic systems. Numerical differentiation is easier but may introduce noise and substantial computational cost. Automatic differentiation provides an important alternative because derivatives can be generated systematically from the computational representation of objectives, dynamics, and constraints.

Interior-point methods approach constrained optimization differently by maintaining iterates within, or progressively toward, the feasible region. Inequality constraints are incorporated using barrier functions that impose increasingly severe penalties as variables approach forbidden boundaries. A sequence of barrier problems is solved while the barrier parameter is reduced, allowing the solution to approach the boundary of the original constrained problem without directly crossing prohibited constraints.

Interior-point methods are particularly useful for large nonlinear programs containing many continuous variables and constraints. Their linear algebra structure can often exploit sparsity produced by trajectory discretization, because a state at one time step usually interacts directly with only nearby states and controls. Efficient sparse factorization can therefore make optimization over long horizons practical even when the complete nonlinear program contains thousands of decision variables.

Collision avoidance is one of the most difficult components of trajectory optimization. A collision-free condition must be converted into a differentiable or approximately differentiable constraint or cost. Signed-distance functions are frequently useful because positive distance represents separation from an obstacle while negative distance represents penetration. Gradients of distance provide the optimizer with information about which direction should move the robot away from collision.

Hard constraints and soft costs serve different purposes. A hard velocity limit, for example, requires every candidate solution to remain within a specified bound, whereas a soft velocity penalty merely discourages excessive speed. Safety-critical requirements are generally better represented as constraints when the solver and formulation permit it. Comfort, smoothness, energy efficiency, and preference objectives are more naturally represented as weighted cost terms.

Optimization-based planning is inherently local for many nonlinear formulations. The solver normally improves an initial trajectory rather than guaranteeing discovery of the globally optimal trajectory. A poor initialization may converge to an undesirable local minimum or fail to find a feasible solution. For this reason, graph search or sampling-based planning can provide an initial path that is subsequently refined through optimization, combining global exploration with continuous trajectory improvement.

Constraint feasibility is often more important than pure objective reduction during early iterations. If the initial trajectory passes through obstacles or violates dynamics, the optimizer must simultaneously recover feasibility and improve trajectory quality. Penalty methods, slack variables, trust regions, merit functions, and filter methods provide mechanisms for balancing these requirements. Robust implementations monitor both objective improvement and constraint violation rather than considering cost alone.

Scaling is another critical numerical issue. Position may be measured in meters, orientation in radians, velocity in meters per second, and forces in hundreds or thousands of newtons. Large differences in numerical magnitude can produce poorly conditioned optimization problems. Normalization and appropriate scaling of decision variables, constraints, and objective terms improve numerical stability and make termination tolerances more meaningful across different physical quantities.

Termination criteria typically examine quantities such as objective change, constraint violation, gradient magnitude, step size, and optimality residuals. A solver reporting convergence does not automatically mean that a trajectory is safe or operationally acceptable. The resulting motion should therefore undergo independent validation for collision clearance, dynamic feasibility, actuator limits, discretization errors, and execution margins before being passed to the robot controller.

For mobile robots, trajectory optimization can jointly determine geometric path and velocity profile while enforcing steering, curvature, acceleration, and obstacle constraints. This differs from planning a geometric path first and assigning speed afterward. Joint optimization becomes particularly valuable for nonholonomic or dynamically constrained platforms because spatial and temporal decisions interact. Narrow turns, braking zones, and limited traction can then influence the trajectory directly.

For manipulators and whole-body robots, the optimization variables may contain dozens of joint positions and velocities at every time step. Constraints can enforce joint limits, end-effector poses, self-collision avoidance, contact conditions, balance, and torque limits. The same mathematical framework therefore extends from relatively simple AMR navigation to manipulation, legged locomotion, humanoid motion, and UAV trajectory generation within the broader optimization-based planning chapter. :chatgpt-content-reference{index="0"}

궤적 최적화(Trajectory Optimization)는 로봇의 움직임 계획(Motion Planning)을 시간에 따른 궤적(Trajectory)을 설명하는 결정 변수(Decision Variable)를 사용하는 연속 최적화 문제(Continuous Optimization Problem)로 정식화한다. 그래프(Graph)를 탐색하거나 구성을 무작위로 샘플링하는 대신, 계획기(Planner)가 상태(State), 제어 입력(Control Input), 또는 궤적 매개변수(Trajectory Parameter)를 직접 수정하여 운동, 충돌, 시스템 제약조건을 만족하면서 목적함수(Objective)를 최소화한다. 이러한 접근법은 경로 품질(Path Quality)과 동역학적 실행 가능성(Dynamic Feasibility)을 자연스럽게 통합할 수 있다.

궤적(Trajectory)은 이산 로봇 상태(Discrete Robot State) \\(x_0,x_1,\\ldots,x_N\\), 제어 입력(Control Input) \\(u_0,u_1,\\ldots,u_{N-1}\\), 또는 스플라인(Spline)과 같은 연속 곡선(Continuous Curve)의 매개변수로 표현할 수 있다. 따라서 최적화 변수(Optimization Variable)에는 위치(Position), 자세(Orientation), 속도(Velocity), 가속도(Acceleration), 조향각(Steering Angle), 관절 구성(Joint Configuration), 시간 간격(Time Interval) 등이 포함될 수 있다. 이러한 표현 방식은 문제의 수치적 복잡성(Numerical Complexity)과 표현 가능한 물리적 제약조건의 종류를 결정한다.

일반적인 궤적 최적화 문제(Trajectory Optimization Problem)는 등식 제약조건(Equality Constraint)과 부등식 제약조건(Inequality Constraint)을 만족하면서 목적함수(Objective Function) \\(J(x,u)\\)를 최소화한다. 등식 제약조건은 일반적으로 초기 및 최종 조건(Initial and Terminal Condition), 로봇 동역학(Robot Dynamics), 운동학적 관계(Kinematic Relationship)를 기술하며, 부등식 제약조건은 충돌 회피(Collision Avoidance), 액추에이터 한계(Actuator Limit), 속도 제한(Velocity Bound), 가속도 제한(Acceleration Limit), 안전 여유(Safety Margin)를 규정한다. 따라서 실제 로봇 계획 문제는 흔히 제약 비선형 계획 문제(Constrained Nonlinear Programming Problem), 즉 NLP로 정식화된다.

목적함수(Objective Function)는 바람직한 로봇 움직임의 특성을 수치적인 비용(Cost)으로 변환한다. 일반적인 비용 항(Cost Term)은 궤적 길이(Trajectory Length), 이동 시간(Travel Time), 에너지 소비(Energy Consumption), 제어 노력(Control Effort), 가속도, 저크(Jerk), 장애물과의 근접성(Proximity to Obstacles), 기준 경로(Reference Path)로부터의 편차 등을 억제한다. 일반적으로 여러 비용 항을 가중 계수(Weighting Coefficient)를 사용하여 결합하며, 최적화기는 모든 요구사항을 절대 조건으로 취급하는 것이 아니라 수학적으로 상충하는 목적 사이의 균형을 찾기 때문에 가중치 선택이 중요하다.

비선형성(Nonlinearity)은 로봇 시스템에서 자연스럽게 발생한다. 로봇 동역학(Robot Dynamics)에는 삼각함수 관계(Trigonometric Relationship)와 상태 및 제어 입력 사이의 곱이 포함되며, 충돌 제약조건(Collision Constraint)은 로봇의 형상과 장애물 경계에 비선형적으로 의존한다. 이동 로봇(Mobile Robot)은 임의의 횡방향 움직임을 허용하지 않는 비홀로노믹 제약조건(Nonholonomic Constraint)을 추가로 가질 수 있다. 매니퓰레이터(Manipulator), 다족 로봇(Legged Robot), 무인항공기(UAV)는 더욱 복잡한 동역학을 가지므로 일반적인 비선형 최적화(Nonlinear Optimization)가 최적화 기반 움직임 계획(Optimization-Based Motion Planning)의 중요한 기반이 된다.

궤적 최적화에서는 연속시간 운동 방정식(Continuous-Time Motion Equation)을 수치 최적화기(Numerical Solver)가 처리할 수 있는 유한 차원의 문제(Finite-Dimensional Problem)로 변환해야 한다. 직접 전사(Direct Transcription)는 선택된 시간 지점에서 궤적을 이산화하고 상태(State)와 제어 입력(Control)을 최적화 변수로 도입한다. 이후 수치 적분(Numerical Integration) 또는 콜로케이션 제약조건(Collocation Constraint)을 이용하여 인접한 지점 사이에서 동역학을 만족시킨다. 세밀한 이산화(Fine Discretization)는 궤적 표현 정확도를 향상시키지만 변수 수와 계산 비용을 증가시킨다.

직접 슈팅(Direct Shooting)은 제어 입력(Control Input)을 주요 최적화 변수로 사용하고 로봇 동역학을 시간에 따라 전방 적분(Forward Integration)하여 상태를 계산한다. 이 방법은 명시적인 상태 변수의 수를 줄일 수 있지만, 초기 제어 입력의 변화가 전체 궤적에 전파되기 때문에 긴 시간 구간(Long Horizon)에서는 수치적 조건(Numerical Conditioning)이 악화될 수 있다. 다중 슈팅(Multiple Shooting)은 시간 구간을 짧은 여러 구간으로 나누고 중간 상태(Intermediate State)를 도입하여 수치적 강건성(Numerical Robustness)을 높이는 대신 구간 사이에 연속성 제약조건(Continuity Constraint)을 추가한다.

순차 이차 계획법(Sequential Quadratic Programming, SQP)은 비선형 제약 최적화 문제를 일련의 이차 계획법(Quadratic Programming, QP) 하위 문제를 통해 해결한다. 각 반복 단계에서 비선형 목적함수는 현재 궤적 주변에서 국소적으로 근사되고, 제약조건은 선형화(Linearization)된다. 이차 계획 하위 문제는 탐색 방향(Search Direction)을 계산하고 이후 궤적을 갱신하며 이러한 과정을 반복한다. SQP는 미분 정보(Derivative Information)와 적절한 초기 궤적(Initial Trajectory)이 제공될 경우 빠른 국소 수렴(Local Convergence)을 달성할 수 있다.

SQP의 효율성은 그래디언트 정보(Gradient Information)와 2차 곡률(Second-Order Curvature)의 근사에 크게 의존한다. 해석적 미분(Analytical Derivative)은 높은 정확도를 제공할 수 있지만 복잡한 로봇 시스템에서는 구현과 유지가 어려워진다. 수치 미분(Numerical Differentiation)은 구현이 간단하지만 잡음(Noise)을 발생시키고 상당한 계산 비용을 요구할 수 있다. 자동 미분(Automatic Differentiation)은 목적함수, 동역학, 제약조건의 계산 표현으로부터 미분값을 체계적으로 생성할 수 있기 때문에 중요한 대안이 된다.

내부점 방법(Interior-Point Method)은 제약 최적화 문제에 대해 SQP와 다른 방식으로 접근하며, 반복해(Iterate)를 실행 가능 영역(Feasible Region) 내부에 유지하거나 점진적으로 접근하도록 한다. 부등식 제약조건은 변수가 금지된 경계에 접근할수록 더욱 큰 페널티를 부과하는 장벽 함수(Barrier Function)를 이용하여 처리한다. 장벽 매개변수(Barrier Parameter)를 점차 감소시키면서 일련의 장벽 문제(Barrier Problem)를 해결함으로써 금지된 제약조건을 직접 넘지 않으면서 원래 제약 문제의 경계에 해가 접근하도록 한다.

내부점 방법(Interior-Point Method)은 많은 연속 변수와 제약조건을 포함하는 대규모 비선형 계획 문제(Large Nonlinear Program)에 특히 유용하다. 궤적 이산화(Trajectory Discretization)에서 생성되는 희소성(Sparsity)을 선형대수 구조에서 활용할 수 있는데, 특정 시간 단계의 상태는 일반적으로 인접한 상태와 제어 입력에만 직접적으로 영향을 받기 때문이다. 따라서 효율적인 희소 행렬 분해(Sparse Factorization)를 이용하면 전체 비선형 계획 문제가 수천 개의 결정 변수를 포함하더라도 긴 시간 구간에 대한 최적화를 실용적으로 수행할 수 있다.

충돌 회피(Collision Avoidance)는 궤적 최적화에서 가장 어려운 구성 요소 중 하나이다. 충돌이 발생하지 않는 조건을 미분 가능하거나 근사적으로 미분 가능한 제약조건 또는 비용으로 변환해야 한다. 부호 거리 함수(Signed-Distance Function)는 양의 거리가 장애물과의 분리를 나타내고 음의 거리가 침투(Penetration)를 나타내기 때문에 자주 활용된다. 거리 그래디언트(Distance Gradient)는 로봇을 충돌 상태에서 벗어나도록 어느 방향으로 이동시켜야 하는지에 대한 정보를 최적화기에 제공한다.

경성 제약조건(Hard Constraint)과 연성 비용(Soft Cost)은 서로 다른 목적을 가진다. 예를 들어 경성 속도 제한(Hard Velocity Limit)은 모든 후보 해가 지정된 한계를 반드시 만족하도록 요구하지만, 연성 속도 페널티(Soft Velocity Penalty)는 과도한 속도를 억제할 뿐 절대적으로 금지하지는 않는다. 안전 필수 요구사항(Safety-Critical Requirement)은 최적화기와 문제 정식화가 허용한다면 제약조건으로 표현하는 것이 일반적으로 적합하다. 승차감, 부드러운 움직임, 에너지 효율, 선호도와 같은 목적은 가중 비용 항(Weighted Cost Term)으로 표현하는 것이 자연스럽다.

많은 비선형 정식화에서 최적화 기반 계획(Optimization-Based Planning)은 본질적으로 국소적(Local)이다. 최적화기는 일반적으로 전역 최적 궤적(Global Optimal Trajectory)의 발견을 보장하는 대신 초기 궤적을 점진적으로 개선한다. 부적절한 초기화는 바람직하지 않은 국소 최솟값(Local Minimum)으로 수렴하거나 실행 가능한 해를 찾지 못하게 할 수 있다. 따라서 그래프 탐색(Graph Search)이나 샘플링 기반 계획(Sampling-Based Planning)을 이용해 초기 경로를 생성하고 이를 최적화를 통해 개선하면 전역 탐색(Global Exploration)과 연속적인 궤적 개선을 결합할 수 있다.

초기 반복 단계에서는 순수한 목적함수 감소보다 제약조건 실행 가능성(Constraint Feasibility)이 더 중요할 수 있다. 초기 궤적이 장애물을 통과하거나 동역학 제약조건을 위반하는 경우 최적화기는 실행 가능성을 회복하는 동시에 궤적 품질도 향상시켜야 한다. 페널티 방법(Penalty Method), 슬랙 변수(Slack Variable), 신뢰 영역(Trust Region), 메리트 함수(Merit Function), 필터 방법(Filter Method)은 이러한 요구사항 사이의 균형을 조절하는 수단을 제공한다. 강건한 구현에서는 비용 감소뿐만 아니라 제약조건 위반 정도도 함께 감시한다.

스케일링(Scaling) 역시 중요한 수치적 문제이다. 위치는 미터(meter), 자세는 라디안(radian), 속도는 초당 미터(meter per second), 힘은 수백 또는 수천 뉴턴(newton) 단위로 표현될 수 있다. 수치적 크기의 큰 차이는 조건이 좋지 않은 최적화 문제(Poorly Conditioned Optimization Problem)를 만들 수 있다. 결정 변수, 제약조건, 목적함수 항을 적절하게 정규화(Normalization)하고 스케일링하면 수치적 안정성이 향상되고 서로 다른 물리량에 대한 종료 허용오차(Termination Tolerance)를 보다 의미 있게 설정할 수 있다.

종료 기준(Termination Criteria)은 일반적으로 목적함수 변화, 제약조건 위반, 그래디언트 크기, 스텝 크기(Step Size), 최적성 잔차(Optimality Residual) 등을 평가한다. 최적화기가 수렴(Convergence)을 보고했다고 해서 해당 궤적이 자동으로 안전하거나 실제 운용에 적합하다는 의미는 아니다. 따라서 생성된 움직임은 로봇 제어기(Robot Controller)에 전달되기 전에 충돌 여유(Collision Clearance), 동역학적 실행 가능성, 액추에이터 한계, 이산화 오차(Discretization Error), 실행 여유(Execution Margin)에 대해 독립적으로 검증되어야 한다.

이동 로봇(Mobile Robot)의 경우 궤적 최적화는 조향(Steering), 곡률(Curvature), 가속도, 장애물 제약조건을 만족시키면서 기하학적 경로(Geometric Path)와 속도 프로파일(Velocity Profile)을 동시에 결정할 수 있다. 이는 먼저 기하학적 경로를 계획하고 이후 속도를 할당하는 방식과 다르다. 비홀로노믹 또는 동역학적 제약을 가진 플랫폼에서는 공간적 결정과 시간적 결정이 서로 영향을 주기 때문에 결합 최적화(Joint Optimization)가 특히 중요하다. 이에 따라 급격한 회전, 제동 구간, 제한된 접지력(Traction)과 같은 조건이 궤적에 직접 반영될 수 있다.

매니퓰레이터(Manipulator)와 전신 로봇(Whole-Body Robot)의 경우 최적화 변수에는 각 시간 단계마다 수십 개의 관절 위치와 속도가 포함될 수 있다. 제약조건을 통해 관절 한계(Joint Limit), 말단 장치 자세(End-Effector Pose), 자기 충돌 회피(Self-Collision Avoidance), 접촉 조건(Contact Condition), 균형(Balance), 토크 한계(Torque Limit)를 적용할 수 있다. 따라서 동일한 수학적 프레임워크(Mathematical Framework)를 비교적 단순한 자율이동로봇(Autonomous Mobile Robot, AMR)의 내비게이션부터 매니퓰레이션(Manipulation), 다족 보행(Legged Locomotion), 휴머노이드 움직임(Humanoid Motion), 무인항공기 궤적 생성(UAV Trajectory Generation)까지 확장할 수 있다.

##  

## 04.02. CHOMP Covariant Hamiltonian Optimization [w/Code]

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

CHOMP, or Covariant Hamiltonian Optimization for Motion Planning, is an optimization-based method that generates smooth, collision-free robot trajectories by iteratively improving an initial path. Rather than constructing motion through graph expansion or random sampling, CHOMP treats the entire trajectory as an optimization variable. This formulation allows geometric smoothness and obstacle avoidance to be considered simultaneously within a continuous trajectory space.

The central idea of CHOMP is to define a trajectory cost containing two major components: a smoothness cost and an obstacle cost. The smoothness term penalizes undesirable variations in velocity or acceleration, while the obstacle term penalizes configurations that approach or intersect obstacles. Optimization modifies the trajectory to reduce the combined cost, gradually transforming an initial path into a smoother and safer motion suitable for robot execution.

A robot trajectory can be represented as a time-parameterized configuration \\(q(t)\\), where each configuration contains the robot\'s degrees of freedom. In numerical implementations, the continuous trajectory is discretized into a sequence of configurations. These trajectory points become optimization variables, while the start and goal configurations are normally fixed. Intermediate configurations are adjusted repeatedly until the trajectory reaches an acceptable local optimum.

The smoothness objective is commonly expressed through derivatives of the trajectory. Penalizing squared velocity encourages short and regular motion, while acceleration penalties suppress abrupt changes in velocity and produce smoother trajectories. Higher-order derivatives may also be considered when additional motion quality is required. In matrix form, these costs create structured quadratic expressions that provide useful curvature information for optimization.

Obstacle avoidance requires information about the robot\'s surrounding workspace rather than only configuration-space collision labels. CHOMP commonly relies on a workspace distance field that provides the distance from spatial locations to nearby obstacles. The distance information can be converted into a differentiable obstacle potential, enabling the optimizer to determine not only whether a trajectory is unsafe but also how the trajectory should move away from obstacles.

For a robot with multiple links, collision cost cannot be evaluated only at a single reference point. Points distributed across the robot body can be mapped from configuration space into workspace through forward kinematics. Each point receives an obstacle cost according to its location in the distance field. The contributions are then propagated back toward configuration variables through kinematic Jacobians, allowing whole-body trajectories to respond to nearby obstacles.

A distinctive feature of CHOMP is its covariant gradient update. Ordinary gradient descent depends strongly on the coordinate representation and may produce inefficient trajectory modifications. CHOMP instead preconditions the functional gradient using a metric related to trajectory smoothness. The resulting update accounts for relationships among neighboring trajectory points, so optimization tends to deform an entire section of the path coherently rather than moving individual waypoints independently.

This metric can be interpreted as defining the geometry of trajectory space. Directions that create highly irregular motion are treated differently from directions that produce smooth deformation. Consequently, the optimizer searches according to a notion of distance that better reflects trajectory quality. This covariant treatment is one of the major conceptual differences between CHOMP and simple waypoint-based gradient descent applied directly in Euclidean coordinates.

The obstacle functional is designed to consider collision exposure along the trajectory rather than merely at isolated waypoints. A useful formulation integrates obstacle potential along the path while accounting for the velocity of workspace points. This makes the cost less dependent on a particular time parameterization and focuses optimization on the geometric shape of the trajectory. Discretization must nevertheless be sufficiently dense to detect collisions between neighboring configurations.

During each optimization iteration, CHOMP evaluates the current trajectory, computes smoothness and obstacle gradients, combines them, and applies a metric-scaled update. Start and goal conditions remain fixed while intermediate configurations move. Repeated updates typically push colliding portions away from obstacles while regularizing unnecessary curvature. Step size and cost weighting determine how aggressively the trajectory changes and therefore strongly influence convergence behavior.

CHOMP is a local optimization method, so the initial trajectory has a major influence on the final solution. A simple initialization may be produced through interpolation between the start and goal configurations, but such a path can pass directly through obstacles. CHOMP can often deform an initially colliding trajectory toward free space, which is an important advantage, although difficult obstacle arrangements may trap the optimization in an undesirable local minimum.

Local minima become particularly important when different homotopy classes exist. If reaching the goal requires moving around a large obstacle in a fundamentally different direction, gradient information near the current trajectory may not reveal that alternative. A global planner or sampling-based method can therefore generate candidate initial paths, after which CHOMP can refine them. This hybrid approach combines broad exploration with continuous optimization of trajectory quality.

The balance between smoothness and obstacle avoidance is controlled through cost parameters. Excessive smoothness weighting may prevent the trajectory from bending sufficiently around obstacles, whereas excessive obstacle weighting can generate unnecessarily large deviations or reduce motion quality. Obstacle potential design, safety distance, update rate, and termination thresholds must therefore be selected according to robot geometry, environment scale, and required clearance.

Distance-field quality directly affects obstacle gradients. A signed or unsigned distance representation should provide sufficiently smooth spatial information near obstacle boundaries so that optimization receives meaningful descent directions. Grid resolution that is too coarse can distort narrow passages or produce inaccurate gradients, while extremely fine resolution increases memory and computation. Practical systems must therefore balance environmental fidelity against planning performance.

CHOMP can be computationally attractive because each iteration uses gradient information and structured trajectory matrices rather than repeatedly performing uninformed search. However, computational requirements increase with the number of trajectory points, robot degrees of freedom, collision-checking points, and environmental complexity. Efficient forward kinematics, Jacobian evaluation, distance-field queries, and matrix operations are therefore important for responsive planning.

Compared with generic nonlinear programming, CHOMP introduces structure specifically designed for motion planning. Smoothness metrics, functional gradients, workspace obstacle potentials, and covariant updates encode characteristics of robot trajectories directly into the optimization process. This specialization can produce high-quality continuous motions efficiently, although it does not eliminate the fundamental challenges of initialization, nonconvex collision geometry, and local optimality.

For mobile robots, CHOMP can refine geometric paths by reducing unnecessary curvature while maintaining clearance from walls and obstacles. For manipulators, it can optimize high-dimensional joint trajectories while considering the swept motion of multiple links. Similar principles can contribute to mobile manipulation and whole-body planning, although additional kinematic, dynamic, balance, or contact constraints may require extensions beyond the basic CHOMP formulation.

In practical motion-planning architecture, CHOMP is best understood as a trajectory refinement mechanism within the broader optimization-based planning family. It demonstrates how continuous obstacle information, trajectory smoothness, robot kinematics, and geometry-aware gradient updates can be integrated into one optimization process. Its core contribution is not simply minimizing path length, but systematically deforming a complete robot trajectory toward smooth, collision-free motion.

CHOMP(Covariant Hamiltonian Optimization for Motion Planning)는 초기 경로(Initial Path)를 반복적으로 개선하여 부드럽고 충돌이 없는 로봇 궤적(Robot Trajectory)을 생성하는 최적화 기반 방법(Optimization-Based Method)이다. 그래프 확장(Graph Expansion)이나 무작위 샘플링(Random Sampling)을 통해 움직임을 구성하는 대신, CHOMP는 전체 궤적(Trajectory)을 하나의 최적화 변수(Optimization Variable)로 취급한다. 이러한 정식화(Formulation)를 통해 기하학적 부드러움(Geometric Smoothness)과 장애물 회피(Obstacle Avoidance)를 연속적인 궤적 공간(Continuous Trajectory Space)에서 동시에 고려할 수 있다.

CHOMP의 핵심 개념은 두 가지 주요 요소인 부드러움 비용(Smoothness Cost)과 장애물 비용(Obstacle Cost)을 포함하는 궤적 비용(Trajectory Cost)을 정의하는 것이다. 부드러움 항(Smoothness Term)은 속도 또는 가속도의 바람직하지 않은 변화를 억제하며, 장애물 항(Obstacle Term)은 장애물에 접근하거나 장애물과 교차하는 구성(Configuration)에 페널티(Penalty)를 부과한다. 최적화 과정은 결합된 비용을 감소시키도록 궤적을 수정하여 초기 경로를 점진적으로 더욱 부드럽고 안전한 로봇 실행용 움직임으로 변환한다.

로봇 궤적(Robot Trajectory)은 시간에 따라 매개변수화된 구성(Time-Parameterized Configuration) \\(q(t)\\)로 표현할 수 있으며, 각 구성에는 로봇의 자유도(Degrees of Freedom)가 포함된다. 수치적 구현(Numerical Implementation)에서는 연속 궤적을 일련의 구성으로 이산화(Discretization)한다. 이러한 궤적 지점(Trajectory Point)이 최적화 변수가 되며 시작 구성(Start Configuration)과 목표 구성(Goal Configuration)은 일반적으로 고정된다. 중간 구성(Intermediate Configuration)은 궤적이 적절한 국소 최적해(Local Optimum)에 도달할 때까지 반복적으로 조정된다.

부드러움 목적함수(Smoothness Objective)는 일반적으로 궤적의 미분값(Derivative)을 이용하여 표현된다. 속도의 제곱(Squared Velocity)에 페널티를 부과하면 짧고 규칙적인 움직임을 유도할 수 있으며, 가속도 페널티(Acceleration Penalty)는 급격한 속도 변화를 억제하여 더욱 부드러운 궤적을 생성한다. 추가적인 움직임 품질(Motion Quality)이 요구되는 경우 고차 미분(Higher-Order Derivative)도 고려할 수 있다. 행렬 형태(Matrix Form)에서는 이러한 비용이 구조화된 이차식(Structured Quadratic Expression)을 형성하여 최적화에 유용한 곡률 정보(Curvature Information)를 제공한다.

장애물 회피(Obstacle Avoidance)를 수행하려면 단순한 구성 공간 충돌 표시(Configuration-Space Collision Label)뿐만 아니라 로봇 주변 작업공간(Workspace)에 대한 정보가 필요하다. CHOMP는 일반적으로 공간상의 위치에서 주변 장애물까지의 거리를 제공하는 작업공간 거리장(Workspace Distance Field)을 활용한다. 이러한 거리 정보는 미분 가능한 장애물 포텐셜(Differentiable Obstacle Potential)로 변환될 수 있으며, 이를 통해 최적화기는 궤적이 안전하지 않은지 판단할 뿐만 아니라 장애물에서 벗어나기 위해 궤적을 어느 방향으로 이동시켜야 하는지도 결정할 수 있다.

여러 링크(Link)를 가진 로봇에서는 하나의 기준점(Reference Point)만으로 충돌 비용을 평가할 수 없다. 로봇 몸체 전체에 분포된 지점들을 순기구학(Forward Kinematics)을 이용하여 구성 공간(Configuration Space)에서 작업공간(Workspace)으로 매핑할 수 있다. 각 지점은 거리장(Distance Field)에서의 위치에 따라 장애물 비용을 부여받는다. 이후 각 비용의 영향은 운동학적 자코비안(Kinematic Jacobian)을 통해 구성 변수(Configuration Variable)로 역전파되어 로봇 전체 몸체의 궤적이 주변 장애물에 대응할 수 있도록 한다.

CHOMP의 특징적인 요소는 공변 그래디언트 갱신(Covariant Gradient Update)이다. 일반적인 그래디언트 하강법(Gradient Descent)은 좌표 표현(Coordinate Representation)에 크게 의존하며 비효율적인 궤적 변경을 발생시킬 수 있다. CHOMP는 대신 궤적의 부드러움과 관련된 메트릭(Metric)을 이용하여 함수 그래디언트(Functional Gradient)를 사전조건화(Preconditioning)한다. 그 결과 생성되는 갱신은 인접한 궤적 지점 사이의 관계를 고려하므로 개별 웨이포인트(Waypoint)를 독립적으로 이동시키기보다 경로의 전체 구간을 일관성 있게 변형하는 경향을 가진다.

이러한 메트릭(Metric)은 궤적 공간(Trajectory Space)의 기하학(Geometry)을 정의하는 것으로 해석할 수 있다. 매우 불규칙한 움직임을 생성하는 방향은 부드러운 변형을 만드는 방향과 다르게 취급된다. 따라서 최적화기는 궤적 품질을 보다 적절하게 반영하는 거리 개념에 따라 탐색을 수행한다. 이러한 공변 처리(Covariant Treatment)는 CHOMP와 유클리드 좌표(Euclidean Coordinate)에서 웨이포인트에 직접 적용되는 단순 그래디언트 하강법(Simple Gradient Descent)을 구분하는 주요 개념적 차이 중 하나이다.

장애물 함수(Obstacle Functional)는 고립된 웨이포인트만을 평가하는 대신 전체 궤적을 따라 충돌에 노출되는 정도를 고려하도록 설계된다. 유용한 정식화에서는 작업공간 지점의 속도를 고려하면서 경로를 따라 장애물 포텐셜(Obstacle Potential)을 적분한다. 이를 통해 비용이 특정 시간 매개변수화(Time Parameterization)에 지나치게 의존하지 않도록 하면서 궤적의 기하학적 형태(Geometric Shape)에 최적화를 집중시킬 수 있다. 그러나 인접한 구성 사이에서 발생하는 충돌을 감지하려면 충분히 조밀한 이산화(Discretization)가 필요하다.

각 최적화 반복(Optimization Iteration)에서 CHOMP는 현재 궤적을 평가하고 부드러움 그래디언트(Smoothness Gradient)와 장애물 그래디언트(Obstacle Gradient)를 계산한 뒤 이를 결합하여 메트릭으로 스케일링된 갱신(Metric-Scaled Update)을 적용한다. 시작 조건과 목표 조건은 고정된 상태로 유지되며 중간 구성만 이동한다. 반복적인 갱신을 통해 충돌하는 궤적 구간은 일반적으로 장애물에서 멀어지고 불필요한 곡률(Curvature)은 완화된다. 스텝 크기(Step Size)와 비용 가중치(Cost Weighting)는 궤적이 얼마나 적극적으로 변화하는지를 결정하므로 수렴 특성(Convergence Behavior)에 큰 영향을 미친다.

CHOMP는 국소 최적화 방법(Local Optimization Method)이므로 초기 궤적(Initial Trajectory)이 최종 해에 큰 영향을 미친다. 단순한 초기화는 시작 구성과 목표 구성 사이의 보간(Interpolation)을 통해 생성할 수 있지만 이러한 경로는 장애물을 직접 통과할 수 있다. CHOMP는 충돌하는 초기 궤적을 자유 공간(Free Space) 방향으로 변형할 수 있는 경우가 많으며 이는 중요한 장점이다. 그러나 복잡한 장애물 배치에서는 최적화가 바람직하지 않은 국소 최솟값(Local Minimum)에 갇힐 수 있다.

서로 다른 호모토피 클래스(Homotopy Class)가 존재하는 경우 국소 최솟값 문제는 특히 중요해진다. 목표에 도달하기 위해 큰 장애물을 완전히 다른 방향으로 우회해야 하는 상황에서는 현재 궤적 주변의 그래디언트 정보만으로 이러한 대안을 발견하지 못할 수 있다. 따라서 전역 계획기(Global Planner) 또는 샘플링 기반 방법(Sampling-Based Method)을 이용하여 후보 초기 경로를 생성한 후 CHOMP를 이용해 이를 정제할 수 있다. 이러한 하이브리드 접근법(Hybrid Approach)은 광범위한 탐색과 연속적인 궤적 품질 최적화를 결합한다.

부드러움과 장애물 회피 사이의 균형은 비용 매개변수(Cost Parameter)를 통해 조절된다. 부드러움 가중치(Smoothness Weight)가 지나치게 높으면 궤적이 장애물을 충분히 우회하도록 굽어지지 않을 수 있으며, 장애물 가중치(Obstacle Weight)가 지나치게 높으면 불필요하게 큰 우회가 발생하거나 움직임 품질이 저하될 수 있다. 따라서 장애물 포텐셜 설계(Obstacle Potential Design), 안전 거리(Safety Distance), 갱신률(Update Rate), 종료 임계값(Termination Threshold)은 로봇의 형상, 환경 규모, 요구되는 안전 여유에 따라 설정해야 한다.

거리장(Distance Field)의 품질은 장애물 그래디언트(Obstacle Gradient)에 직접적인 영향을 준다. 부호 거리 표현(Signed Distance Representation) 또는 비부호 거리 표현(Unsigned Distance Representation)은 장애물 경계 주변에서 충분히 부드러운 공간 정보를 제공하여 최적화기가 의미 있는 하강 방향(Descent Direction)을 얻을 수 있도록 해야 한다. 격자 해상도(Grid Resolution)가 너무 낮으면 좁은 통로(Narrow Passage)가 왜곡되거나 부정확한 그래디언트가 생성될 수 있으며, 지나치게 높은 해상도는 메모리와 계산량을 증가시킨다. 따라서 실제 시스템에서는 환경 표현 정확도와 계획 성능 사이의 균형을 고려해야 한다.

CHOMP는 각 반복 과정에서 비정보적 탐색(Uninformed Search)을 반복하는 대신 그래디언트 정보와 구조화된 궤적 행렬(Structured Trajectory Matrix)을 사용하므로 계산 측면에서 효율적일 수 있다. 그러나 궤적 지점의 수, 로봇의 자유도, 충돌 검사 지점(Collision-Checking Point)의 수, 환경 복잡성이 증가하면 계산 요구량도 증가한다. 따라서 빠른 계획을 위해서는 효율적인 순기구학(Forward Kinematics), 자코비안 평가(Jacobian Evaluation), 거리장 질의(Distance-Field Query), 행렬 연산(Matrix Operation)이 중요하다.

일반적인 비선형 계획법(Nonlinear Programming)과 비교하면 CHOMP는 움직임 계획(Motion Planning)을 위해 특별히 설계된 구조를 도입한다. 부드러움 메트릭(Smoothness Metric), 함수 그래디언트(Functional Gradient), 작업공간 장애물 포텐셜(Workspace Obstacle Potential), 공변 갱신(Covariant Update)은 로봇 궤적의 특성을 최적화 과정에 직접 반영한다. 이러한 특화 구조를 통해 높은 품질의 연속적인 움직임을 효율적으로 생성할 수 있지만 초기화, 비볼록 충돌 기하학(Nonconvex Collision Geometry), 국소 최적성(Local Optimality)이라는 근본적인 문제까지 제거하는 것은 아니다.

이동 로봇(Mobile Robot)의 경우 CHOMP는 벽과 장애물로부터 안전 거리를 유지하면서 불필요한 곡률을 감소시켜 기하학적 경로(Geometric Path)를 정제할 수 있다. 매니퓰레이터(Manipulator)의 경우 여러 링크의 스윕 움직임(Swept Motion)을 고려하면서 고차원 관절 궤적(High-Dimensional Joint Trajectory)을 최적화할 수 있다. 유사한 원리는 이동 매니퓰레이션(Mobile Manipulation)과 전신 계획(Whole-Body Planning)에도 적용할 수 있지만 추가적인 운동학, 동역학, 균형, 접촉 제약조건을 처리하기 위해서는 기본 CHOMP 정식화 이상의 확장이 필요할 수 있다.

실제 움직임 계획 아키텍처(Motion-Planning Architecture)에서 CHOMP는 보다 광범위한 최적화 기반 계획(Optimization-Based Planning) 계열에 속하는 궤적 정제 메커니즘(Trajectory Refinement Mechanism)으로 이해하는 것이 적절하다. CHOMP는 연속적인 장애물 정보, 궤적 부드러움, 로봇 운동학, 기하학을 고려한 그래디언트 갱신(Geometry-Aware Gradient Update)을 하나의 최적화 과정으로 통합할 수 있음을 보여준다. 핵심적인 기여는 단순히 경로 길이를 최소화하는 것이 아니라 전체 로봇 궤적을 체계적으로 변형하여 부드럽고 충돌이 없는 움직임으로 만드는 데 있다.

##  

## 04.03. TrajOpt Sequential Convex Optimization [w/Code]

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

TrajOpt, short for Trajectory Optimization, is an optimization-based motion planning framework that formulates robot trajectory generation as a sequence of convex optimization problems. Within the optimization-based planning structure, it follows general nonlinear trajectory optimization and CHOMP while introducing sequential convex optimization as its central mechanism. The method is particularly useful for generating smooth, collision-free trajectories in high-dimensional robot configuration spaces.

The fundamental difficulty in robot trajectory optimization is that collision avoidance, robot kinematics, and some motion constraints are nonlinear and often nonconvex. Solving the complete problem directly can therefore be computationally expensive and sensitive to initialization. TrajOpt addresses this difficulty by approximating the original nonconvex problem locally with convex subproblems that can be solved efficiently using established numerical optimization techniques.

A trajectory is usually represented as a finite sequence of robot configurations \\(q_0,q_1,\\ldots,q_T\\). The start and goal configurations may be fixed, while intermediate configurations become decision variables. Depending on the robot and task, additional variables can represent velocities, accelerations, timing, or auxiliary quantities. Optimization then modifies these variables simultaneously instead of independently selecting individual waypoints.

The objective function typically combines trajectory smoothness, motion efficiency, collision avoidance, and task-specific preferences. Smoothness can be encouraged by penalizing differences between consecutive configurations or changes in velocity and acceleration. Additional costs may penalize deviation from preferred poses, excessive joint displacement, or undesirable end-effector motion. The relative weights determine how strongly each property influences the resulting trajectory.

Sequential convex optimization begins from an initial trajectory and constructs a local convex approximation around the current solution. Nonlinear objective terms may be approximated by convex functions, while nonlinear constraints are linearized or otherwise convexified. The resulting convex subproblem is solved to obtain an improved trajectory. This trajectory becomes the expansion point for the next approximation, and the procedure continues iteratively.

The local nature of each approximation is controlled using a trust region. A convex approximation is generally reliable only near the trajectory around which it was constructed, so allowing excessively large updates can invalidate the model. Trust-region constraints restrict how far the new trajectory may move during one iteration. Their size can be adjusted according to whether predicted improvements agree with improvements measured using the original nonlinear problem.

Collision avoidance is a major contribution of the TrajOpt formulation. Rather than relying only on binary collision tests at discrete configurations, the optimization uses distance-related information between robot geometry and environmental obstacles. Signed-distance or closest-point information can provide gradients indicating how robot links should move to increase clearance. Collision conditions can then be locally approximated and incorporated into the convex optimization process.

Checking collision only at trajectory waypoints can miss collisions occurring between two configurations. TrajOpt therefore emphasizes continuous-time collision checking, where the swept motion between consecutive trajectory states is considered. This is especially important when trajectory discretization is relatively coarse or robot links move significantly between adjacent states. Continuous checking reduces the possibility that an apparently feasible discrete trajectory actually intersects an obstacle during execution.

Robot geometry is commonly represented through collision primitives, meshes, or convex components. For each relevant robot-obstacle pair, closest points and separation distances can be calculated. The associated contact normals provide directions for increasing separation. These geometric quantities allow collision avoidance to be expressed through locally linearized constraints or penalty terms that can be processed efficiently inside sequential convex optimization.

Hard constraints and soft penalties can both appear in TrajOpt. Joint limits, fixed start states, required poses, and certain kinematic relationships may be imposed as explicit constraints. Collision avoidance or task preferences may also be represented through penalty functions when strict feasibility is difficult to maintain during early iterations. This flexibility enables the optimizer to recover progressively from an initially infeasible or colliding trajectory.

Penalty-based formulations often use hinge-like costs that become active when a required safety distance is violated. If the robot remains farther than the specified margin from an obstacle, the collision penalty can become zero. As the robot approaches or penetrates the safety region, the penalty increases. This structure concentrates optimization effort near potentially dangerous portions of the trajectory while avoiding unnecessary cost in clearly collision-free regions.

Sequential convex optimization differs from solving one fixed convex problem. After every iteration, robot configurations change, collision geometry changes, and the local approximations must be recomputed. Thus, TrajOpt repeatedly alternates between evaluating the nonlinear robotic problem and solving a tractable convex approximation. Convergence occurs when trajectory changes, objective improvement, and constraint violations become sufficiently small according to selected termination criteria.

Initialization remains important because the complete motion-planning problem is nonconvex even though each subproblem is convex. A straight-line interpolation in configuration space is a common simple initialization and may initially intersect obstacles. TrajOpt can often repair moderate collisions through local optimization, but it cannot guarantee escape from every poor local minimum or discover a fundamentally different route around a large obstacle.

For environments containing multiple distinct route classes, a global planner can therefore complement TrajOpt. Graph-based or sampling-based planning can first identify a collision-free or approximately feasible route, after which TrajOpt refines it for smoothness, clearance, and task constraints. This division allows global planning to handle large topological decisions while trajectory optimization handles continuous geometric improvement.

Compared with CHOMP, TrajOpt shares the principle of improving an entire trajectory through optimization but uses a different numerical strategy. CHOMP is strongly associated with functional gradients, trajectory-space metrics, and covariant updates, whereas TrajOpt emphasizes sequential convex approximations, trust regions, and explicit treatment of constraints. Both remain local optimization approaches whose performance depends on trajectory initialization and environmental geometry.

Convex subproblems are attractive because they can be solved reliably and efficiently relative to general nonconvex optimization. Depending on the formulation, a subproblem may resemble quadratic programming or another convex program. Sparsity also arises naturally because costs and constraints at one trajectory step often depend primarily on neighboring states. Exploiting this structure is important when optimizing long trajectories or robots with many degrees of freedom.

Manipulator planning is a major application because arm trajectories may contain many joint variables while collision constraints involve multiple moving links. TrajOpt can simultaneously adjust all intermediate joint configurations to avoid obstacles and self-collision while maintaining smooth motion. End-effector position and orientation requirements can also be introduced, making the framework useful for reaching, manipulation, and constrained motion tasks.

The same principles can be extended to mobile robots and mobile manipulators. Configuration variables may include planar position, heading, steering quantities, manipulator joints, or other platform states. Appropriate constraints can represent nonholonomic motion, velocity bounds, workspace restrictions, and task conditions. More complex dynamic systems require additional modeling when forces, torques, contacts, or full equations of motion must be respected.

Practical performance depends on trajectory resolution, collision geometry, safety margins, trust-region parameters, penalty coefficients, solver tolerances, and initialization quality. Excessively coarse discretization can reduce geometric accuracy, while unnecessarily dense trajectories increase computation. Similarly, overly aggressive updates can invalidate local approximations, whereas excessively restrictive trust regions may cause slow convergence. Parameter selection therefore balances robustness, accuracy, and planning latency.

A trajectory returned by TrajOpt should still be independently validated before execution. Collision checking should verify both discrete configurations and continuous motion between them, while joint, velocity, acceleration, and task constraints should be checked against the actual robot model. Optimization convergence indicates numerical termination, not automatically operational safety, so execution margins and controller compatibility remain essential parts of the planning pipeline.

TrajOpt demonstrates an important principle of optimization-based motion planning: a difficult nonconvex robotic problem can be approached through a sequence of simpler local problems. By combining trajectory discretization, collision-distance information, continuous collision checking, convex approximation, and trust-region updates, sequential convex optimization provides a practical bridge between general nonlinear optimization and computationally efficient robot trajectory generation.

TrajOpt(Trajectory Optimization)는 로봇 궤적 생성(Robot Trajectory Generation)을 일련의 볼록 최적화 문제(Convex Optimization Problem)로 정식화하는 최적화 기반 움직임 계획 프레임워크(Optimization-Based Motion Planning Framework)이다. 최적화 기반 계획(Optimization-Based Planning)의 구조에서 일반적인 비선형 궤적 최적화(Nonlinear Trajectory Optimization)와 CHOMP에 이어 순차 볼록 최적화(Sequential Convex Optimization)를 핵심 메커니즘으로 도입한다. 이 방법은 특히 고차원 로봇 구성 공간(High-Dimensional Robot Configuration Space)에서 부드럽고 충돌이 없는 궤적을 생성하는 데 유용하다.

로봇 궤적 최적화(Robot Trajectory Optimization)의 근본적인 어려움은 충돌 회피(Collision Avoidance), 로봇 운동학(Robot Kinematics), 그리고 일부 움직임 제약조건(Motion Constraint)이 비선형(Nonlinear)이며 흔히 비볼록(Nonconvex)이라는 점이다. 따라서 전체 문제를 직접 해결하면 계산 비용이 매우 높고 초기화(Initialization)에 민감할 수 있다. TrajOpt는 원래의 비볼록 문제를 국소적으로 볼록 하위 문제(Convex Subproblem)로 근사하여 기존의 수치 최적화 기법(Numerical Optimization Technique)으로 효율적으로 해결함으로써 이러한 어려움을 완화한다.

궤적(Trajectory)은 일반적으로 유한한 로봇 구성(Robot Configuration)의 연속인 \\(q_0,q_1,\\ldots,q_T\\)로 표현된다. 시작 구성(Start Configuration)과 목표 구성(Goal Configuration)은 고정할 수 있으며, 중간 구성(Intermediate Configuration)은 결정 변수(Decision Variable)가 된다. 로봇과 작업에 따라 추가적인 변수로 속도(Velocity), 가속도(Acceleration), 시간(Timing), 또는 보조 변수(Auxiliary Quantity)를 표현할 수 있다. 최적화 과정은 개별 웨이포인트(Waypoint)를 독립적으로 선택하는 대신 이러한 변수들을 동시에 수정한다.

목적함수(Objective Function)는 일반적으로 궤적 부드러움(Trajectory Smoothness), 움직임 효율성(Motion Efficiency), 충돌 회피(Collision Avoidance), 작업별 선호 조건(Task-Specific Preference)을 결합한다. 연속된 구성 사이의 차이나 속도 및 가속도의 변화를 억제하여 부드러운 움직임을 유도할 수 있다. 추가 비용 항은 선호 자세(Preferred Pose)로부터의 편차, 과도한 관절 변위(Joint Displacement), 바람직하지 않은 말단 장치 움직임(End-Effector Motion)을 억제할 수 있다. 각 항의 상대적인 가중치는 결과 궤적에 각 특성이 얼마나 강하게 반영되는지를 결정한다.

순차 볼록 최적화(Sequential Convex Optimization)는 초기 궤적에서 시작하여 현재 해(Current Solution) 주변에 국소적인 볼록 근사(Local Convex Approximation)를 구성한다. 비선형 목적함수 항은 볼록 함수(Convex Function)로 근사할 수 있으며, 비선형 제약조건은 선형화(Linearization)하거나 다른 방식으로 볼록화(Convexification)한다. 이렇게 생성된 볼록 하위 문제를 해결하여 개선된 궤적을 얻는다. 이 궤적은 다음 근사의 기준점(Expansion Point)이 되며 이러한 절차가 반복적으로 수행된다.

각 근사의 국소적인 특성은 신뢰 영역(Trust Region)을 이용하여 제어한다. 볼록 근사는 일반적으로 근사가 생성된 궤적 주변에서만 신뢰할 수 있기 때문에 지나치게 큰 갱신을 허용하면 근사 모델의 유효성이 떨어질 수 있다. 신뢰 영역 제약조건(Trust-Region Constraint)은 한 번의 반복에서 새로운 궤적이 이동할 수 있는 범위를 제한한다. 예측된 개선량과 원래 비선형 문제에서 실제로 측정된 개선량이 얼마나 일치하는지에 따라 신뢰 영역의 크기를 조절할 수 있다.

충돌 회피(Collision Avoidance)는 TrajOpt 정식화의 중요한 요소이다. 이 최적화 방법은 이산적인 구성에서의 이진 충돌 검사(Binary Collision Test)에만 의존하는 대신 로봇 형상(Robot Geometry)과 환경 장애물(Environmental Obstacle) 사이의 거리 관련 정보를 사용한다. 부호 거리(Signed Distance) 또는 최근접점 정보(Closest-Point Information)는 로봇 링크가 안전 여유를 증가시키기 위해 어느 방향으로 움직여야 하는지를 나타내는 그래디언트(Gradient)를 제공할 수 있다. 이후 충돌 조건을 국소적으로 근사하여 볼록 최적화 과정에 포함할 수 있다.

궤적 웨이포인트에서만 충돌을 검사하면 두 구성 사이에서 발생하는 충돌을 놓칠 수 있다. 따라서 TrajOpt는 연속시간 충돌 검사(Continuous-Time Collision Checking)를 중요하게 다루며, 연속된 두 궤적 상태 사이에서 로봇이 이동하며 차지하는 스윕 움직임(Swept Motion)을 고려한다. 이는 궤적 이산화(Trajectory Discretization)가 비교적 성기거나 인접 상태 사이에서 로봇 링크가 크게 움직이는 경우 특히 중요하다. 연속 검사는 겉보기에는 실행 가능한 이산 궤적이 실제 실행 중 장애물과 충돌할 가능성을 감소시킨다.

로봇 형상(Robot Geometry)은 일반적으로 충돌 프리미티브(Collision Primitive), 메시(Mesh), 또는 볼록 구성 요소(Convex Component)를 이용하여 표현된다. 관련된 각 로봇-장애물 쌍(Robot-Obstacle Pair)에 대해 최근접점과 분리 거리(Separation Distance)를 계산할 수 있다. 이에 대응하는 접촉 법선(Contact Normal)은 두 물체 사이의 거리를 증가시키는 방향을 제공한다. 이러한 기하학적 정보를 이용하여 충돌 회피를 국소적으로 선형화된 제약조건이나 페널티 항(Penalty Term)으로 표현하고 순차 볼록 최적화 내부에서 효율적으로 처리할 수 있다.

TrajOpt에서는 경성 제약조건(Hard Constraint)과 연성 페널티(Soft Penalty)를 모두 사용할 수 있다. 관절 한계(Joint Limit), 고정된 시작 상태(Fixed Start State), 요구 자세(Required Pose), 특정 운동학적 관계(Kinematic Relationship)는 명시적인 제약조건으로 적용할 수 있다. 초기 반복 단계에서 엄격한 실행 가능성(Strict Feasibility)을 유지하기 어려운 경우 충돌 회피나 작업 선호 조건을 페널티 함수(Penalty Function)로 표현할 수도 있다. 이러한 유연성은 최적화기가 초기의 실행 불가능하거나 충돌하는 궤적에서 점진적으로 회복할 수 있도록 한다.

페널티 기반 정식화(Penalty-Based Formulation)는 요구되는 안전 거리(Safety Distance)가 위반될 때 활성화되는 힌지 형태 비용(Hinge-Like Cost)을 사용하는 경우가 많다. 로봇이 장애물로부터 지정된 여유 거리보다 멀리 떨어져 있으면 충돌 페널티는 0이 될 수 있다. 로봇이 안전 영역에 접근하거나 침투할수록 페널티는 증가한다. 이러한 구조는 명확하게 충돌이 없는 영역에서는 불필요한 비용을 발생시키지 않으면서 잠재적으로 위험한 궤적 구간에 최적화 노력을 집중시킨다.

순차 볼록 최적화는 하나의 고정된 볼록 문제를 해결하는 것과 다르다. 각 반복 이후 로봇 구성은 변화하고 충돌 기하학(Collision Geometry)도 변화하므로 국소 근사를 다시 계산해야 한다. 따라서 TrajOpt는 비선형 로봇 문제를 평가하는 과정과 계산 가능한 볼록 근사 문제를 해결하는 과정을 반복적으로 수행한다. 궤적 변화량, 목적함수 개선량, 제약조건 위반 정도가 설정된 종료 기준(Termination Criteria)에 따라 충분히 작아지면 수렴(Convergence)한 것으로 판단한다.

각 하위 문제는 볼록하지만 전체 움직임 계획 문제는 여전히 비볼록이기 때문에 초기화(Initialization)는 중요하다. 구성 공간에서 시작점과 목표점 사이를 직선 보간(Straight-Line Interpolation)하는 방법이 일반적인 단순 초기화 방식이며 초기 궤적은 장애물을 통과할 수도 있다. TrajOpt는 국소 최적화를 통해 비교적 작은 충돌을 수정할 수 있지만 모든 불량한 국소 최솟값(Poor Local Minimum)에서 탈출하거나 큰 장애물을 완전히 다른 방향으로 우회하는 경로를 발견할 수 있다고 보장하지는 않는다.

서로 다른 여러 경로 클래스(Route Class)가 존재하는 환경에서는 전역 계획기(Global Planner)가 TrajOpt를 보완할 수 있다. 그래프 기반 계획(Graph-Based Planning)이나 샘플링 기반 계획(Sampling-Based Planning)을 이용하여 먼저 충돌이 없거나 대략적으로 실행 가능한 경로를 탐색하고 이후 TrajOpt를 이용해 부드러움, 안전 여유, 작업 제약조건을 개선할 수 있다. 이러한 역할 분담은 전역 계획이 큰 위상학적 결정(Topological Decision)을 처리하고 궤적 최적화가 연속적인 기하학적 개선을 담당하도록 한다.

CHOMP와 비교하면 TrajOpt 역시 전체 궤적을 최적화를 통해 개선한다는 원리를 공유하지만 서로 다른 수치적 전략(Numerical Strategy)을 사용한다. CHOMP는 함수 그래디언트(Functional Gradient), 궤적 공간 메트릭(Trajectory-Space Metric), 공변 갱신(Covariant Update)과 밀접하게 관련되는 반면, TrajOpt는 순차 볼록 근사(Sequential Convex Approximation), 신뢰 영역(Trust Region), 명시적인 제약조건 처리(Explicit Constraint Handling)를 강조한다. 두 방법 모두 국소 최적화 접근법이며 성능은 궤적 초기화와 환경의 기하학적 구조에 영향을 받는다.

볼록 하위 문제(Convex Subproblem)는 일반적인 비볼록 최적화(Nonconvex Optimization)에 비해 신뢰성 있고 효율적으로 해결할 수 있다는 장점이 있다. 정식화에 따라 각 하위 문제는 이차 계획법(Quadratic Programming) 또는 다른 형태의 볼록 계획법(Convex Programming)과 유사할 수 있다. 또한 특정 궤적 단계의 비용과 제약조건은 주로 인접 상태에 의존하므로 자연스럽게 희소성(Sparsity)이 발생한다. 긴 궤적이나 자유도가 많은 로봇을 최적화할 때 이러한 구조를 활용하는 것이 중요하다.

매니퓰레이터 계획(Manipulator Planning)은 TrajOpt의 주요 응용 분야이다. 로봇 팔의 궤적에는 많은 관절 변수가 포함될 수 있으며 충돌 제약조건은 여러 개의 움직이는 링크와 관련된다. TrajOpt는 모든 중간 관절 구성(Intermediate Joint Configuration)을 동시에 조정하여 장애물 및 자기 충돌(Self-Collision)을 회피하면서 부드러운 움직임을 유지할 수 있다. 말단 장치의 위치와 자세 요구조건도 추가할 수 있으므로 도달(Reaching), 매니퓰레이션(Manipulation), 제약 움직임(Constrained Motion) 작업에 활용할 수 있다.

동일한 원리는 이동 로봇(Mobile Robot)과 이동 매니퓰레이터(Mobile Manipulator)에도 확장할 수 있다. 구성 변수에는 평면 위치(Planar Position), 헤딩(Heading), 조향 변수(Steering Quantity), 매니퓰레이터 관절, 기타 플랫폼 상태가 포함될 수 있다. 적절한 제약조건을 통해 비홀로노믹 움직임(Nonholonomic Motion), 속도 제한(Velocity Bound), 작업공간 제한(Workspace Restriction), 작업 조건(Task Condition)을 표현할 수 있다. 힘, 토크, 접촉, 전체 운동방정식까지 고려해야 하는 복잡한 동적 시스템에서는 추가적인 모델링이 필요하다.

실제 성능은 궤적 해상도(Trajectory Resolution), 충돌 형상(Collision Geometry), 안전 여유(Safety Margin), 신뢰 영역 매개변수(Trust-Region Parameter), 페널티 계수(Penalty Coefficient), 최적화기 허용오차(Solver Tolerance), 초기화 품질에 영향을 받는다. 지나치게 성긴 이산화는 기하학적 정확도를 떨어뜨리는 반면 불필요하게 조밀한 궤적은 계산량을 증가시킨다. 지나치게 공격적인 갱신은 국소 근사를 무효화할 수 있고, 너무 제한적인 신뢰 영역은 수렴을 느리게 만들 수 있다. 따라서 매개변수 설정에서는 강건성(Robustness), 정확도(Accuracy), 계획 지연시간(Planning Latency)의 균형이 필요하다.

TrajOpt가 생성한 궤적은 실제 실행 전에 독립적으로 검증해야 한다. 충돌 검사(Collision Checking)를 통해 이산 구성뿐만 아니라 구성 사이의 연속적인 움직임도 검증해야 하며, 관절, 속도, 가속도, 작업 제약조건 역시 실제 로봇 모델을 기준으로 확인해야 한다. 최적화 수렴은 수치적 종료(Numerical Termination)를 의미할 뿐 자동적으로 운용 안전성(Operational Safety)을 보장하지 않으므로 실행 여유(Execution Margin)와 제어기 호환성(Controller Compatibility) 역시 계획 파이프라인의 필수 요소이다.

TrajOpt는 최적화 기반 움직임 계획(Optimization-Based Motion Planning)의 중요한 원리를 보여준다. 즉, 어려운 비볼록 로봇 문제를 일련의 더 단순한 국소 문제(Local Problem)를 통해 해결할 수 있다는 것이다. 궤적 이산화, 충돌 거리 정보(Collision-Distance Information), 연속 충돌 검사, 볼록 근사(Convex Approximation), 신뢰 영역 갱신(Trust-Region Update)을 결합함으로써 순차 볼록 최적화는 일반적인 비선형 최적화와 계산 효율적인 로봇 궤적 생성 사이를 연결하는 실용적인 방법을 제공한다.

##  

## 04.04. iLQR Iterative LQR for Robot Trajectory Opt [w/Code]

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

Iterative Linear Quadratic Regulator, or iLQR, is a trajectory optimization method for nonlinear dynamical systems that repeatedly solves locally approximated linear-quadratic control problems. In robot planning, it optimizes a sequence of control inputs and the resulting state trajectory together. This makes iLQR especially useful when motion quality depends not only on geometric paths but also on robot dynamics, control effort, and time-dependent behavior.

The method begins with a discrete-time nonlinear system described by \\(x_{t+1}=f(x_t,u_t)\\), where \\(x_t\\) represents the robot state and \\(u_t\\) represents the control input. The state may contain position, orientation, velocity, or joint variables, while controls may represent forces, torques, steering commands, or accelerations. A candidate control sequence generates a corresponding trajectory through forward simulation of the dynamics.

Trajectory quality is expressed using a finite-horizon objective containing running costs and a terminal cost. A typical formulation is \\(J=\\sum_{t=0}\^{T-1}l(x_t,u_t)+l_f(x_T)\\). Running costs can penalize control effort, state deviation, velocity, obstacle proximity, or undesirable motion, while the terminal cost encourages the final state to approach the desired goal. Weighting terms determine the relative importance of these objectives.

The original trajectory optimization problem is difficult because both the dynamics and cost functions may be nonlinear. iLQR handles this by repeatedly constructing a local approximation around a nominal trajectory. The nonlinear dynamics are linearized with respect to states and controls, while the cost function is approximated locally by a quadratic model. This produces a structure similar to a finite-horizon Linear Quadratic Regulator problem.

Around a nominal state-control pair \\((\\bar{x}_t,\\bar{u}_t)\\), small perturbations satisfy approximately \\(\\delta x_{t+1}=A_t\\delta x_t+B_t\\delta u_t\\). Here, \\(A_t\\) and \\(B_t\\) are Jacobians of the dynamics with respect to state and control. These matrices describe how small changes propagate through the robot dynamics and provide the local linear model required for the optimization step.

The quadratic cost approximation describes how the objective changes near the nominal trajectory. First-order derivatives represent local cost gradients, while second-order terms capture curvature with respect to states and controls. Together with the linearized dynamics, these quantities allow iLQR to determine locally improved control commands without solving the complete nonlinear optimization problem directly at every iteration.

A defining feature of iLQR is its backward pass. Starting from the terminal time and proceeding toward the initial state, the algorithm recursively approximates the value function and computes control updates. Each update normally contains a feedforward component and a feedback component. The feedforward term improves the nominal control sequence, while the feedback gain specifies how controls should respond to deviations from the nominal state trajectory.

The backward recursion is closely related to the Riccati recursion used in LQR. At each time step, derivatives of the local action-value function are assembled from immediate cost information and the value approximation at the next state. Minimizing this local quadratic model with respect to control produces an optimal local control increment and a state-feedback gain, which are then propagated backward through the planning horizon.

After the backward pass, iLQR performs a forward pass through the actual nonlinear dynamics. Starting from the initial state, new controls are generated using the calculated feedforward updates and feedback gains. The resulting states are obtained through nonlinear forward simulation rather than through the linear approximation. This produces a new dynamically consistent nominal trajectory that can be evaluated using the original objective function.

A line-search procedure is commonly applied during the forward pass. The full feedforward update may be too aggressive because the local quadratic model is accurate only near the nominal trajectory. Scaling the update by a parameter such as \\(\\alpha\\) allows several candidate trajectories to be tested. An update is accepted when it provides sufficient cost reduction, improving numerical robustness and helping prevent divergence.

The backward and forward passes are repeated until the objective improvement, control change, or another convergence measure becomes sufficiently small. Each iteration therefore follows a recognizable cycle: simulate the nominal trajectory, locally approximate dynamics and costs, perform backward dynamic programming, generate a new trajectory through forward rollout, evaluate it, and repeat. This structure gives iLQR its characteristic computational efficiency.

One important strength of iLQR is that dynamics are embedded directly into trajectory generation. A geometric planner may produce a collision-free path that cannot be followed because of acceleration, steering, or dynamic limitations. In contrast, iLQR generates trajectories through the system dynamics, so resulting motions naturally reflect properties such as inertia, control authority, turning behavior, and coupling between state variables.

The distinction between iLQR and Differential Dynamic Programming, or DDP, is important. Both methods use backward and forward passes over a nonlinear trajectory. Standard iLQR typically linearizes the dynamics and uses a second-order approximation of the cost while neglecting second derivatives of the dynamics. Full DDP additionally incorporates those dynamics second derivatives, potentially improving local modeling at the cost of greater computational complexity.

Obstacle avoidance can be incorporated through state-dependent costs that increase as the robot approaches obstacles. Distance fields, signed-distance functions, or other differentiable geometric representations can provide gradients for this purpose. However, obstacle avoidance expressed purely as a soft cost does not automatically guarantee collision-free motion. Strong penalties, constraint extensions, or independent collision validation may therefore be required for safety-critical applications.

Control and state limits introduce another practical challenge. Basic iLQR is naturally formulated around unconstrained local quadratic control updates, whereas robots have actuator saturation, velocity limits, steering limits, joint bounds, and other restrictions. Practical variants may clamp controls, use constrained backward-pass methods, transform variables, or incorporate penalties and augmented optimization techniques to handle these restrictions more explicitly.

Initialization affects convergence because iLQR is fundamentally a local trajectory optimization method. An initial control sequence may consist of zeros, simple heuristic controls, a stabilized trajectory, or commands derived from another planner. Poor initialization can place the optimization in an undesirable local minimum or prevent it from reaching the desired state. Warm starting from a previously optimized trajectory is especially valuable in repeated planning.

Computational efficiency results from exploiting the temporal structure of the control problem. Rather than treating every state and control variable as an unrelated decision variable in a generic nonlinear program, iLQR performs structured recursions along the trajectory horizon. For systems with moderate state and control dimensions, this can provide rapid trajectory improvement and makes the method attractive for iterative planning and model-predictive applications.

For mobile robots, iLQR can optimize steering and acceleration while respecting nonlinear vehicle or differential-drive dynamics. For manipulators, states may include joint positions and velocities while controls represent torques or other actuation commands. Legged robots, autonomous vehicles, and UAVs can similarly use dynamics-aware trajectory optimization, although contact-rich or highly constrained systems often require more sophisticated extensions.

iLQR is also closely related to Model Predictive Control, or MPC. An optimized finite-horizon trajectory can be computed repeatedly as the robot moves, with only the first control command applied before the optimization is solved again using a new measured state. Warm-starting from the previous solution can reduce computation. This receding-horizon strategy allows trajectory optimization to adapt continuously to state estimation errors and environmental changes.

Successful implementation depends on accurate dynamics, derivative computation, cost design, numerical regularization, horizon length, time discretization, and convergence criteria. Analytical derivatives, automatic differentiation, or numerical derivatives may be used to construct local models. Regularization is often required when matrices in the backward pass become poorly conditioned, ensuring that control updates remain numerically stable and produce meaningful descent directions.

Within optimization-based robot planning, iLQR provides a bridge between optimal control and trajectory generation. Its central mechanism combines nonlinear forward simulation with local linearization, quadratic cost approximation, Riccati-style backward recursion, feedback control updates, and repeated rollout. The result is a practical framework for producing dynamically feasible trajectories while jointly optimizing robot states, controls, and motion quality.

반복 선형 이차 조절기(Iterative Linear Quadratic Regulator, iLQR)는 비선형 동적 시스템(Nonlinear Dynamical System)을 위한 궤적 최적화(Trajectory Optimization) 방법으로, 국소적으로 근사된 선형-이차 제어 문제(Linear-Quadratic Control Problem)를 반복적으로 해결한다. 로봇 계획(Robot Planning)에서 iLQR은 일련의 제어 입력(Control Input)과 그 결과로 생성되는 상태 궤적(State Trajectory)을 함께 최적화한다. 따라서 움직임 품질이 기하학적 경로뿐만 아니라 로봇 동역학(Robot Dynamics), 제어 노력(Control Effort), 시간 의존적 거동(Time-Dependent Behavior)에 영향을 받는 경우 특히 유용하다.

이 방법은 \\(x_{t+1}=f(x_t,u_t)\\)로 표현되는 이산시간 비선형 시스템(Discrete-Time Nonlinear System)에서 시작하며, 여기서 \\(x_t\\)는 로봇 상태(Robot State), \\(u_t\\)는 제어 입력(Control Input)을 나타낸다. 상태에는 위치(Position), 자세(Orientation), 속도(Velocity), 관절 변수(Joint Variable) 등이 포함될 수 있으며, 제어 입력은 힘(Force), 토크(Torque), 조향 명령(Steering Command), 가속도(Acceleration) 등을 나타낼 수 있다. 후보 제어 입력 시퀀스(Candidate Control Sequence)는 동역학의 전방 시뮬레이션(Forward Simulation)을 통해 이에 대응하는 궤적을 생성한다.

궤적 품질(Trajectory Quality)은 구간 비용(Running Cost)과 종단 비용(Terminal Cost)을 포함하는 유한 시간 구간 목적함수(Finite-Horizon Objective)를 사용하여 표현한다. 일반적인 정식화는 \\(J=\\sum_{t=0}\^{T-1}l(x_t,u_t)+l_f(x_T)\\)이다. 구간 비용은 제어 노력, 상태 편차(State Deviation), 속도, 장애물 근접도(Obstacle Proximity), 바람직하지 않은 움직임 등을 억제할 수 있으며, 종단 비용은 최종 상태가 원하는 목표에 접근하도록 유도한다. 가중 항(Weighting Term)은 이러한 목적들의 상대적 중요도를 결정한다.

원래의 궤적 최적화 문제는 동역학과 비용함수(Cost Function)가 모두 비선형일 수 있기 때문에 해결하기 어렵다. iLQR은 기준 궤적(Nominal Trajectory) 주변에서 국소 근사(Local Approximation)를 반복적으로 구성하여 이러한 문제를 처리한다. 비선형 동역학은 상태와 제어 입력에 대해 선형화(Linearization)하고, 비용함수는 국소적인 이차 모델(Quadratic Model)로 근사한다. 이를 통해 유한 시간 구간 선형 이차 조절기(Linear Quadratic Regulator, LQR) 문제와 유사한 구조가 만들어진다.

기준 상태-제어 쌍(Nominal State-Control Pair) \\((\\bar{x}_t,\\bar{u}_t)\\) 주변에서 작은 섭동(Perturbation)은 근사적으로 \\(\\delta x_{t+1}=A_t\\delta x_t+B_t\\delta u_t\\)를 만족한다. 여기서 \\(A_t\\)와 \\(B_t\\)는 각각 상태와 제어 입력에 대한 동역학의 자코비안(Jacobian)이다. 이 행렬들은 작은 변화가 로봇 동역학을 통해 어떻게 전파되는지를 나타내며 최적화 단계에 필요한 국소 선형 모델(Local Linear Model)을 제공한다.

이차 비용 근사(Quadratic Cost Approximation)는 기준 궤적 주변에서 목적함수가 어떻게 변화하는지를 나타낸다. 1차 미분(First-Order Derivative)은 국소 비용 그래디언트(Local Cost Gradient)를 나타내며, 2차 항(Second-Order Term)은 상태와 제어 입력에 대한 곡률(Curvature)을 포착한다. 이러한 정보와 선형화된 동역학을 결합하면 iLQR은 매 반복마다 전체 비선형 최적화 문제를 직접 해결하지 않고도 국소적으로 개선된 제어 명령을 계산할 수 있다.

iLQR의 대표적인 특징은 역방향 패스(Backward Pass)이다. 종단 시점(Terminal Time)에서 시작하여 초기 상태 방향으로 진행하면서 알고리즘은 가치함수(Value Function)를 재귀적으로 근사하고 제어 입력 갱신(Control Update)을 계산한다. 각 갱신은 일반적으로 피드포워드 성분(Feedforward Component)과 피드백 성분(Feedback Component)을 포함한다. 피드포워드 항은 기준 제어 시퀀스를 개선하고, 피드백 이득(Feedback Gain)은 기준 상태 궤적으로부터 발생하는 편차에 제어 입력이 어떻게 대응해야 하는지를 결정한다.

역방향 재귀(Backward Recursion)는 LQR에서 사용하는 리카티 재귀(Riccati Recursion)와 밀접한 관련이 있다. 각 시간 단계에서 국소 행동가치함수(Action-Value Function)의 미분값은 현재 비용 정보와 다음 상태의 가치 근사(Value Approximation)를 이용하여 구성된다. 이 국소 이차 모델을 제어 입력에 대해 최소화하면 최적의 국소 제어 증가량(Local Control Increment)과 상태 피드백 이득(State-Feedback Gain)을 얻을 수 있으며, 이러한 정보는 계획 시간 구간을 따라 역방향으로 전파된다.

역방향 패스가 완료되면 iLQR은 실제 비선형 동역학(Nonlinear Dynamics)을 이용하여 전방 패스(Forward Pass)를 수행한다. 초기 상태에서 시작하여 계산된 피드포워드 갱신과 피드백 이득을 이용해 새로운 제어 입력을 생성한다. 이후 상태는 선형 근사가 아니라 비선형 전방 시뮬레이션(Nonlinear Forward Simulation)을 통해 계산된다. 이를 통해 동역학적으로 일관된 새로운 기준 궤적(Dynamically Consistent Nominal Trajectory)을 생성하고 원래의 목적함수로 평가할 수 있다.

전방 패스에서는 일반적으로 선 탐색(Line Search) 절차를 적용한다. 국소 이차 모델은 기준 궤적 주변에서만 정확하기 때문에 전체 피드포워드 갱신을 한 번에 적용하면 지나치게 공격적인 변화가 발생할 수 있다. \\(\\alpha\\)와 같은 매개변수를 이용하여 갱신 크기를 조절하면 여러 후보 궤적을 시험할 수 있다. 충분한 비용 감소(Cost Reduction)를 제공하는 갱신을 선택함으로써 수치적 강건성(Numerical Robustness)을 향상시키고 발산(Divergence)을 방지할 수 있다.

역방향 패스와 전방 패스는 목적함수 개선량, 제어 입력 변화량 또는 다른 수렴 척도(Convergence Measure)가 충분히 작아질 때까지 반복된다. 따라서 각 반복은 기준 궤적 시뮬레이션, 동역학 및 비용의 국소 근사, 역방향 동적 계획법(Backward Dynamic Programming), 전방 롤아웃(Forward Rollout)을 통한 새로운 궤적 생성, 평가 및 재반복이라는 명확한 순환 구조를 가진다. 이러한 구조는 iLQR의 특징적인 계산 효율성을 제공한다.

iLQR의 중요한 장점 중 하나는 동역학(Dynamics)이 궤적 생성 과정에 직접 포함된다는 점이다. 기하학적 계획기(Geometric Planner)는 충돌이 없는 경로를 생성하더라도 가속도, 조향 또는 동역학적 한계 때문에 실제 로봇이 따라갈 수 없는 경로를 만들 수 있다. 반면 iLQR은 시스템 동역학을 통해 궤적을 생성하므로 결과 움직임에는 관성(Inertia), 제어 권한(Control Authority), 회전 특성(Turning Behavior), 상태 변수 사이의 결합(Coupling)과 같은 특성이 자연스럽게 반영된다.

iLQR과 미분 동적 계획법(Differential Dynamic Programming, DDP)의 차이는 중요하다. 두 방법 모두 비선형 궤적에 대해 역방향 및 전방 패스를 사용한다. 표준 iLQR은 일반적으로 동역학을 선형화하고 비용함수에 대한 2차 근사를 사용하면서 동역학의 2차 미분(Second Derivative of Dynamics)은 무시한다. 완전한 DDP는 이러한 동역학의 2차 미분까지 추가로 포함하여 더 정확한 국소 모델링이 가능하지만 계산 복잡도(Computational Complexity)는 증가한다.

장애물 회피(Obstacle Avoidance)는 로봇이 장애물에 접근할수록 증가하는 상태 의존적 비용(State-Dependent Cost)을 통해 포함할 수 있다. 거리장(Distance Field), 부호 거리 함수(Signed-Distance Function), 또는 기타 미분 가능한 기하학적 표현(Differentiable Geometric Representation)을 이용하여 이에 필요한 그래디언트를 제공할 수 있다. 그러나 순수하게 연성 비용(Soft Cost)으로 표현된 장애물 회피는 자동으로 충돌 없는 움직임을 보장하지 않는다. 따라서 안전 필수 응용(Safety-Critical Application)에서는 강한 페널티, 제약조건 확장 또는 독립적인 충돌 검증이 필요할 수 있다.

제어 및 상태 제한(Control and State Limit)은 또 다른 실용적인 문제를 발생시킨다. 기본적인 iLQR은 본질적으로 비제약 국소 이차 제어 갱신(Unconstrained Local Quadratic Control Update)을 중심으로 정식화되지만 실제 로봇에는 액추에이터 포화(Actuator Saturation), 속도 제한, 조향 한계, 관절 한계(Joint Bound) 등의 제약이 존재한다. 실제 구현에서는 제어 입력 제한(Clamping), 제약 역방향 패스(Constrained Backward Pass), 변수 변환(Variable Transformation), 페널티 및 확장 최적화 기법을 사용하여 이러한 제약조건을 보다 명시적으로 처리할 수 있다.

iLQR은 근본적으로 국소 궤적 최적화 방법(Local Trajectory Optimization Method)이기 때문에 초기화(Initialization)는 수렴에 영향을 준다. 초기 제어 시퀀스는 영 입력(Zero Input), 단순한 휴리스틱 제어(Heuristic Control), 안정화된 궤적(Stabilized Trajectory), 또는 다른 계획기에서 생성된 명령으로 구성할 수 있다. 부적절한 초기화는 최적화를 바람직하지 않은 국소 최솟값(Local Minimum)에 빠뜨리거나 목표 상태에 도달하지 못하게 할 수 있다. 반복적인 계획에서는 이전에 최적화된 궤적을 이용하는 웜 스타트(Warm Start)가 특히 유용하다.

계산 효율성(Computational Efficiency)은 제어 문제의 시간적 구조(Temporal Structure)를 활용함으로써 얻어진다. 일반적인 비선형 계획법(Nonlinear Programming)처럼 모든 상태와 제어 변수를 서로 관련 없는 결정 변수로 처리하는 대신 iLQR은 궤적 시간 구간을 따라 구조화된 재귀 계산(Structured Recursion)을 수행한다. 상태와 제어 차원이 중간 규모인 시스템에서는 빠른 궤적 개선이 가능하므로 반복적 계획(Iterative Planning)과 모델 예측 응용(Model-Predictive Application)에 적합하다.

이동 로봇(Mobile Robot)의 경우 iLQR은 비선형 차량 또는 차동구동 동역학(Differential-Drive Dynamics)을 고려하면서 조향과 가속도를 최적화할 수 있다. 매니퓰레이터(Manipulator)에서는 상태에 관절 위치와 속도를 포함하고 제어 입력은 토크 또는 다른 구동 명령을 나타낼 수 있다. 다족 로봇(Legged Robot), 자율주행 차량(Autonomous Vehicle), 무인항공기(UAV)에서도 동역학을 고려한 궤적 최적화에 활용할 수 있지만 접촉이 많거나 제약조건이 복잡한 시스템에는 더욱 정교한 확장이 필요한 경우가 많다.

iLQR은 모델 예측 제어(Model Predictive Control, MPC)와도 밀접하게 관련된다. 로봇이 이동하는 동안 유한 시간 구간의 최적 궤적을 반복적으로 계산하고 첫 번째 제어 명령만 적용한 후 새롭게 측정된 상태를 이용하여 다시 최적화할 수 있다. 이전 해를 이용한 웜 스타트는 계산량을 줄이는 데 도움이 된다. 이러한 이동 시간 구간 전략(Receding-Horizon Strategy)을 통해 궤적 최적화는 상태 추정 오차(State Estimation Error)와 환경 변화에 지속적으로 적응할 수 있다.

성공적인 구현은 정확한 동역학 모델, 미분 계산(Derivative Computation), 비용 설계(Cost Design), 수치 정규화(Numerical Regularization), 계획 시간 구간(Horizon Length), 시간 이산화(Time Discretization), 수렴 기준(Convergence Criteria)에 영향을 받는다. 국소 모델을 구성하기 위해 해석적 미분(Analytical Derivative), 자동 미분(Automatic Differentiation), 수치 미분(Numerical Derivative)을 사용할 수 있다. 역방향 패스의 행렬이 수치적으로 불안정해지는 경우에는 정규화(Regularization)를 적용하여 제어 갱신이 안정적이고 의미 있는 하강 방향(Descent Direction)을 생성하도록 해야 한다.

최적화 기반 로봇 계획(Optimization-Based Robot Planning)에서 iLQR은 최적 제어(Optimal Control)와 궤적 생성(Trajectory Generation)을 연결하는 역할을 한다. 핵심 메커니즘은 비선형 전방 시뮬레이션, 국소 선형화(Local Linearization), 이차 비용 근사, 리카티 방식의 역방향 재귀(Riccati-Style Backward Recursion), 피드백 제어 갱신(Feedback Control Update), 반복적인 롤아웃을 결합한다. 이를 통해 로봇의 상태, 제어 입력, 움직임 품질을 함께 최적화하면서 동역학적으로 실행 가능한 궤적(Dynamically Feasible Trajectory)을 생성하는 실용적인 프레임워크를 제공한다.

##  

## 04.05. Model Predictive Path Integral MPPI Planning [w/Code]

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

Model Predictive Path Integral, or MPPI, planning is a sampling-based stochastic optimal control method that repeatedly evaluates many candidate control sequences over a finite prediction horizon. Instead of relying on explicit gradients of the dynamics or solving a constrained nonlinear program at every cycle, MPPI perturbs a nominal control sequence, simulates the resulting trajectories, evaluates their costs, and uses cost-weighted sampling to improve the controls.

The method considers a dynamical system whose state evolves according to a model such as \\(x_{t+1}=f(x_t,u_t)\\), potentially including stochastic disturbances. The state may contain robot position, orientation, velocity, steering state, or other dynamic variables, while the control vector may represent linear and angular velocity, acceleration, steering commands, forces, or torques. Forward simulation predicts how each candidate control sequence moves the robot.

MPPI operates over a finite prediction horizon containing \\(T\\) future control steps. A nominal sequence \\(U=(u_0,u_1,\\ldots,u_{T-1})\\) represents the current best estimate of future controls. At each optimization cycle, random perturbations are added to this sequence to produce many candidate control sequences. Each candidate is rolled forward through the robot model, generating a complete predicted state trajectory.

The trajectory cost defines the behavior desired from the planner. Typical terms penalize distance from a reference path or goal, obstacle proximity, collision, control effort, excessive velocity, heading error, unstable motion, or violation of preferred operating regions. A terminal cost can encourage progress toward the destination at the end of the horizon. The cost formulation therefore converts navigation objectives and operational preferences into numerical trajectory scores.

Unlike deterministic optimization methods that follow a single local gradient, MPPI evaluates a population of sampled trajectories. Some samples may remain close to the nominal trajectory, while others explore substantially different controls. Low-cost rollouts provide evidence about promising motion directions, whereas high-cost rollouts contribute little to the update. This population-based mechanism allows MPPI to handle nonlinear dynamics and complex cost landscapes without requiring cost derivatives.

A common sampling strategy generates perturbations \\(\\epsilon_t\^k\\) for rollout \\(k\\) from a Gaussian distribution with a selected covariance. Candidate controls can then be written approximately as \\(u_t\^k=u_t+\\epsilon_t\^k\\). The covariance determines the exploration scale: small noise produces conservative local search, while large noise explores a wider region but may generate many inefficient or infeasible trajectories.

Each sampled control sequence is propagated through the robot dynamics to calculate a rollout \\(x_0\^k,x_1\^k,\\ldots,x_T\^k\\). Its total trajectory cost \\(S_k\\) is then evaluated. Thousands of such rollouts may be processed during one planning cycle. Because individual rollouts are largely independent, the simulation and cost evaluation stages are highly parallelizable and can exploit GPU hardware effectively.

The path-integral update converts trajectory costs into importance weights. A commonly used form assigns a weight proportional to \\(\\exp(-S_k/\\lambda)\\), after suitable numerical normalization, where \\(\\lambda\\) is a temperature parameter. Low-cost trajectories receive exponentially larger weights than poor trajectories. The nominal control sequence is updated using the weighted average of sampled control perturbations, directing future controls toward regions associated with better rollouts.

The temperature parameter \\(\\lambda\\) strongly affects optimization behavior. A small value concentrates weight on a limited number of the best trajectories and produces aggressive selection, while a larger value distributes influence across more samples and creates a smoother update. Together with sampling covariance, the temperature determines the balance between exploration, exploitation, responsiveness, and robustness of the controller.

MPPI naturally follows a receding-horizon strategy. After optimization, the robot normally executes only the first control command or a short prefix of the optimized sequence. At the next control cycle, a new state estimate is obtained, the prediction horizon is shifted forward, and optimization is repeated. The unused portion of the previous solution can be shifted and reused as a warm start for the next iteration.

This repeated replanning makes MPPI suitable for environments where the robot state or surrounding scene changes continuously. Localization errors, disturbances, moving obstacles, and deviations from the predicted trajectory can be incorporated at the next optimization cycle. The planner therefore behaves as a feedback process even though individual sampled rollouts primarily evaluate open-loop control sequences over the prediction horizon.

Obstacle avoidance is commonly represented through trajectory costs. A costmap, occupancy grid, signed-distance representation, or geometric collision model can assign increasingly large penalties as predicted states approach obstacles. Direct collision can receive a very large cost so that unsafe rollouts obtain negligible importance weights. Additional inflation or clearance costs can encourage trajectories that preserve a practical safety margin around obstacles.

Hard constraints require careful treatment because basic MPPI is fundamentally cost driven. Velocity, acceleration, steering, and actuator limits can often be enforced by clipping sampled controls or rejecting invalid samples. More complicated state constraints may be represented through large penalties, barrier-like costs, or specialized constrained variants. Independent safety checking remains important because finite sampling does not mathematically guarantee that every executed trajectory is collision-free.

Sampling quality is central to MPPI performance. Too few rollouts may provide an inaccurate estimate of useful control directions, especially in cluttered or highly nonlinear environments. Increasing the number of samples improves coverage but raises computational demand. The prediction horizon creates a similar tradeoff: a longer horizon allows earlier anticipation of turns and obstacles, while increasing the amount of simulation required for every sampled trajectory.

GPU acceleration is especially well matched to MPPI because hundreds or thousands of trajectories can be simulated simultaneously. Robot dynamics, obstacle costs, path-tracking errors, and trajectory reductions can be evaluated using parallel kernels. This differs from algorithms dominated by sequential matrix factorizations or backward recursions. With suitable implementation, parallel rollout evaluation can support high-frequency local planning for dynamically constrained robots.

The robot model determines the physical realism of predicted trajectories. Simple mobile platforms may use differential-drive or unicycle models, while car-like robots require steering and curvature dynamics. More complex applications can employ richer dynamic models containing acceleration, actuator response, or additional states. A more accurate model improves prediction but increases computation, so model complexity must be balanced against the required planning frequency.

Reference-path tracking is a common use of MPPI in navigation architectures. A global planner first generates a broad route toward the goal, while MPPI operates locally to optimize dynamically executable motion along that route. Costs can measure distance and orientation relative to the reference path while simultaneously considering obstacles and control smoothness. The resulting planner therefore combines global guidance with short-horizon reactive optimization.

MPPI differs fundamentally from iLQR in how it searches for improved controls. iLQR linearizes dynamics, approximates costs quadratically, and uses a Riccati-style backward pass to compute local updates. MPPI instead performs stochastic rollouts and obtains updates from cost-weighted samples. It therefore avoids explicit dynamics derivatives but usually requires many model evaluations, making parallel computing particularly important.

Compared with purely geometric local planning, MPPI can reason directly about controls and predicted motion over time. It can evaluate alternative velocity and steering sequences before they are executed rather than selecting motion from instantaneous geometry alone. This capability is valuable when acceleration limits, momentum, turning radius, control smoothness, and short-term obstacle interactions significantly affect whether a candidate motion can actually be executed.

Practical tuning involves the number of rollouts, horizon length, control frequency, sampling covariance, temperature, control bounds, cost weights, and obstacle penalties. These parameters interact strongly. Excessive noise can create unstable candidate controls, while insufficient exploration can trap the planner around a poor nominal solution. Poorly balanced costs can cause oscillation, excessive clearance, weak path tracking, or overly conservative behavior.

The quality of MPPI also depends on computational latency. All sampling, rollout simulation, cost evaluation, weighting, and control updates must finish before the next command is required. Average execution time alone is insufficient for a real robot; latency variation and worst-case behavior matter because delayed commands reduce the validity of predictions. Efficient memory layout, parallelization, and predictable execution are therefore important implementation concerns.

For mobile robots, autonomous vehicles, UAVs, and other dynamically constrained platforms, MPPI provides a flexible framework for local trajectory generation and control. Its core cycle is conceptually simple: sample control perturbations, simulate future trajectories, evaluate their costs, weight promising samples, update the nominal controls, execute the first action, and repeat. Through this receding-horizon process, MPPI combines stochastic exploration, optimal-control reasoning, and real-time feedback planning.

모델 예측 경로 적분(Model Predictive Path Integral, MPPI) 계획은 유한 예측 구간(Finite Prediction Horizon)에 걸쳐 다수의 후보 제어 시퀀스(Candidate Control Sequence)를 반복적으로 평가하는 샘플링 기반 확률적 최적 제어(Sampling-Based Stochastic Optimal Control) 방법이다. MPPI는 동역학의 명시적인 그래디언트(Explicit Gradient)에 의존하거나 매 제어 주기마다 제약 비선형 계획 문제를 해결하는 대신, 기준 제어 시퀀스(Nominal Control Sequence)에 섭동을 추가하고 생성된 궤적을 시뮬레이션한 후 비용을 평가하여 비용 가중 샘플링(Cost-Weighted Sampling)을 통해 제어 입력을 개선한다.

이 방법은 \\(x_{t+1}=f(x_t,u_t)\\)와 같은 모델에 따라 상태가 변화하는 동적 시스템(Dynamical System)을 고려하며, 확률적 외란(Stochastic Disturbance)을 포함할 수도 있다. 상태에는 로봇의 위치(Position), 자세(Orientation), 속도(Velocity), 조향 상태(Steering State) 또는 기타 동적 변수가 포함될 수 있으며, 제어 벡터(Control Vector)는 선속도와 각속도, 가속도, 조향 명령, 힘 또는 토크 등을 나타낼 수 있다. 전방 시뮬레이션(Forward Simulation)은 각 후보 제어 시퀀스가 로봇을 어떻게 움직이는지 예측한다.

MPPI는 \\(T\\)개의 미래 제어 단계를 포함하는 유한 예측 구간(Finite Prediction Horizon)에서 동작한다. 기준 시퀀스 \\(U=(u_0,u_1,\\ldots,u_{T-1})\\)는 현재 추정된 최선의 미래 제어 입력을 나타낸다. 각 최적화 주기(Optimization Cycle)에서 이 시퀀스에 무작위 섭동(Random Perturbation)을 추가하여 다수의 후보 제어 시퀀스를 생성한다. 각 후보는 로봇 모델을 통해 전방 롤아웃(Forward Rollout)되어 완전한 예측 상태 궤적(Predicted State Trajectory)을 생성한다.

궤적 비용(Trajectory Cost)은 계획기가 추구해야 하는 동작을 정의한다. 일반적인 비용 항은 기준 경로(Reference Path) 또는 목표로부터의 거리, 장애물 근접도(Obstacle Proximity), 충돌(Collision), 제어 노력(Control Effort), 과도한 속도, 헤딩 오차(Heading Error), 불안정한 움직임, 선호 운용 영역(Preferred Operating Region)의 위반 등을 억제한다. 종단 비용(Terminal Cost)은 예측 구간 끝에서 목적지 방향으로 진행하도록 유도할 수 있다. 따라서 비용 정식화(Cost Formulation)는 내비게이션 목표와 운용 선호 조건을 수치적인 궤적 점수로 변환한다.

단일 국소 그래디언트를 따라가는 결정론적 최적화 방법(Deterministic Optimization Method)과 달리 MPPI는 샘플링된 궤적 집단(Population of Sampled Trajectories)을 평가한다. 일부 샘플은 기준 궤적 가까이에 머무는 반면 다른 샘플은 상당히 다른 제어 입력을 탐색할 수 있다. 낮은 비용의 롤아웃은 유망한 움직임 방향에 대한 정보를 제공하고 높은 비용의 롤아웃은 갱신에 거의 기여하지 않는다. 이러한 집단 기반 메커니즘(Population-Based Mechanism)을 통해 MPPI는 비용 미분(Cost Derivative)을 요구하지 않고도 비선형 동역학과 복잡한 비용 지형(Cost Landscape)을 처리할 수 있다.

일반적인 샘플링 전략(Sampling Strategy)은 롤아웃 \\(k\\)에 대한 섭동 \\(\\epsilon_t\^k\\)를 선택된 공분산(Covariance)을 갖는 가우시안 분포(Gaussian Distribution)에서 생성한다. 후보 제어 입력은 대략 \\(u_t\^k=u_t+\\epsilon_t\^k\\)로 표현할 수 있다. 공분산은 탐색 범위(Exploration Scale)를 결정한다. 작은 잡음은 보수적인 국소 탐색(Local Search)을 생성하고, 큰 잡음은 더 넓은 영역을 탐색하지만 비효율적이거나 실행 불가능한 궤적을 다수 생성할 수 있다.

각 샘플링된 제어 시퀀스는 로봇 동역학을 통해 전파되어 롤아웃 \\(x_0\^k,x_1\^k,\\ldots,x_T\^k\\)를 생성한다. 이후 전체 궤적 비용 \\(S_k\\)를 평가한다. 하나의 계획 주기에서 수천 개의 롤아웃을 처리할 수도 있다. 각각의 롤아웃은 대부분 서로 독립적이므로 시뮬레이션과 비용 평가 단계는 높은 병렬화 가능성(Parallelizability)을 가지며 그래픽 처리 장치(GPU) 하드웨어를 효과적으로 활용할 수 있다.

경로 적분 갱신(Path-Integral Update)은 궤적 비용을 중요도 가중치(Importance Weight)로 변환한다. 일반적으로 적절한 수치 정규화(Numerical Normalization)를 수행한 후 \\(\\exp(-S_k/\\lambda)\\)에 비례하는 가중치를 부여하며, 여기서 \\(\\lambda\\)는 온도 매개변수(Temperature Parameter)이다. 비용이 낮은 궤적은 좋지 않은 궤적보다 지수적으로 큰 가중치를 받는다. 기준 제어 시퀀스는 샘플링된 제어 섭동의 가중 평균(Weighted Average)을 이용하여 갱신되며, 이를 통해 미래 제어 입력이 더 우수한 롤아웃과 관련된 영역으로 이동한다.

온도 매개변수(Temperature Parameter) \\(\\lambda\\)는 최적화 거동에 큰 영향을 미친다. 작은 값은 소수의 최상위 궤적에 가중치를 집중시켜 공격적인 선택을 만들며, 큰 값은 더 많은 샘플에 영향력을 분산시켜 부드러운 갱신을 생성한다. 온도 매개변수는 샘플링 공분산(Sampling Covariance)과 함께 탐색(Exploration), 활용(Exploitation), 응답성(Responsiveness), 강건성(Robustness) 사이의 균형을 결정한다.

MPPI는 자연스럽게 이동 시간 구간 전략(Receding-Horizon Strategy)을 따른다. 최적화가 완료되면 로봇은 일반적으로 최적화된 시퀀스의 첫 번째 제어 명령 또는 짧은 앞부분만 실행한다. 다음 제어 주기에서 새로운 상태 추정(State Estimate)을 얻고 예측 구간을 앞으로 이동시킨 뒤 최적화를 다시 수행한다. 이전 해에서 아직 사용하지 않은 부분은 이동시켜 다음 반복의 웜 스타트(Warm Start)로 재사용할 수 있다.

이러한 반복적 재계획(Replanning)은 로봇 상태나 주변 환경이 지속적으로 변화하는 상황에서 MPPI를 효과적으로 사용할 수 있게 한다. 위치추정 오차(Localization Error), 외란(Disturbance), 이동 장애물(Moving Obstacle), 예측 궤적으로부터의 편차는 다음 최적화 주기에 반영할 수 있다. 따라서 개별 샘플링 롤아웃이 주로 예측 구간의 개방루프 제어 시퀀스(Open-Loop Control Sequence)를 평가하더라도 전체 계획기는 피드백 과정(Feedback Process)처럼 동작한다.

장애물 회피(Obstacle Avoidance)는 일반적으로 궤적 비용을 통해 표현된다. 비용맵(Costmap), 점유 격자(Occupancy Grid), 부호 거리 표현(Signed-Distance Representation), 기하학적 충돌 모델(Geometric Collision Model)은 예측 상태가 장애물에 접근할수록 증가하는 페널티를 부여할 수 있다. 직접적인 충돌에는 매우 큰 비용을 부과하여 안전하지 않은 롤아웃의 중요도 가중치가 거의 0이 되도록 할 수 있다. 추가적인 팽창 비용(Inflation Cost)이나 여유 거리 비용(Clearance Cost)을 사용하여 장애물 주변에서 실용적인 안전 여유를 유지하도록 유도할 수도 있다.

기본적인 MPPI는 근본적으로 비용 기반(Cost-Driven)이므로 경성 제약조건(Hard Constraint)을 처리할 때 주의가 필요하다. 속도, 가속도, 조향, 액추에이터 한계는 샘플링된 제어 입력을 제한(Clipping)하거나 유효하지 않은 샘플을 제거함으로써 적용할 수 있다. 더 복잡한 상태 제약조건(State Constraint)은 큰 페널티, 장벽 형태 비용(Barrier-Like Cost), 또는 특수한 제약 MPPI 변형을 통해 표현할 수 있다. 유한한 샘플링은 실행되는 모든 궤적의 충돌 회피를 수학적으로 보장하지 않으므로 독립적인 안전 검사가 중요하다.

샘플링 품질(Sampling Quality)은 MPPI 성능의 핵심 요소이다. 롤아웃 수가 너무 적으면 특히 복잡하거나 비선형성이 높은 환경에서 유용한 제어 방향을 부정확하게 추정할 수 있다. 샘플 수를 증가시키면 탐색 범위가 개선되지만 계산 요구량도 증가한다. 예측 구간 역시 유사한 절충 관계를 가진다. 긴 예측 구간은 회전과 장애물을 더 일찍 예측할 수 있지만 각각의 샘플 궤적에 대해 더 많은 시뮬레이션이 필요하다.

그래픽 처리 장치 가속(GPU Acceleration)은 수백 또는 수천 개의 궤적을 동시에 시뮬레이션할 수 있기 때문에 MPPI와 특히 잘 맞는다. 로봇 동역학, 장애물 비용, 경로 추종 오차(Path-Tracking Error), 궤적 축약 연산(Trajectory Reduction)을 병렬 커널(Parallel Kernel)을 이용하여 평가할 수 있다. 이는 순차적인 행렬 분해(Matrix Factorization)나 역방향 재귀(Backward Recursion)가 계산의 중심이 되는 알고리즘과 차이가 있다. 적절하게 구현하면 병렬 롤아웃 평가를 통해 동역학적 제약을 가진 로봇에서도 고주파 국소 계획(High-Frequency Local Planning)을 수행할 수 있다.

로봇 모델(Robot Model)은 예측 궤적의 물리적 현실성(Physical Realism)을 결정한다. 단순한 이동 플랫폼은 차동구동 모델(Differential-Drive Model)이나 유니사이클 모델(Unicycle Model)을 사용할 수 있으며, 자동차형 로봇(Car-Like Robot)은 조향 및 곡률 동역학(Steering and Curvature Dynamics)을 필요로 한다. 더 복잡한 응용에서는 가속도, 액추에이터 응답(Actuator Response), 추가적인 상태를 포함하는 상세 동역학 모델을 사용할 수 있다. 정확한 모델은 예측 성능을 향상시키지만 계산량을 증가시키므로 모델 복잡성과 요구되는 계획 주파수 사이의 균형이 필요하다.

기준 경로 추종(Reference-Path Tracking)은 내비게이션 아키텍처에서 MPPI의 대표적인 활용 방식이다. 전역 계획기(Global Planner)가 먼저 목표까지의 전체적인 경로를 생성하고 MPPI는 국소적으로 동작하면서 해당 경로를 따라 동역학적으로 실행 가능한 움직임을 최적화한다. 비용함수는 장애물과 제어 부드러움을 동시에 고려하면서 기준 경로에 대한 거리와 자세 차이를 측정할 수 있다. 따라서 전역 안내(Global Guidance)와 단기 예측 기반 반응형 최적화(Reactive Optimization)를 결합할 수 있다.

MPPI는 개선된 제어 입력을 탐색하는 방식에서 iLQR과 근본적으로 다르다. iLQR은 동역학을 선형화하고 비용함수를 이차 근사한 뒤 리카티 방식 역방향 패스(Riccati-Style Backward Pass)를 이용하여 국소 갱신을 계산한다. 반면 MPPI는 확률적 롤아웃(Stochastic Rollout)을 수행하고 비용 가중 샘플로부터 제어 갱신을 계산한다. 따라서 명시적인 동역학 미분을 요구하지 않지만 일반적으로 많은 모델 평가가 필요하므로 병렬 컴퓨팅(Parallel Computing)이 특히 중요하다.

순수한 기하학적 국소 계획(Geometric Local Planning)과 비교하면 MPPI는 시간에 따른 제어 입력과 예측 움직임을 직접 고려할 수 있다. 순간적인 기하학 정보만으로 움직임을 선택하는 대신 실행 전에 여러 속도 및 조향 시퀀스를 평가할 수 있다. 이러한 능력은 가속도 한계, 관성(Momentum), 회전 반경(Turning Radius), 제어 부드러움(Control Smoothness), 단기적인 장애물 상호작용이 후보 움직임의 실제 실행 가능성에 큰 영향을 주는 상황에서 특히 유용하다.

실제 튜닝(Practical Tuning)에는 롤아웃 수(Number of Rollouts), 예측 구간 길이(Horizon Length), 제어 주파수(Control Frequency), 샘플링 공분산, 온도, 제어 한계(Control Bound), 비용 가중치(Cost Weight), 장애물 페널티 등이 포함된다. 이러한 매개변수들은 서로 강하게 상호작용한다. 지나치게 큰 잡음은 불안정한 후보 제어를 생성할 수 있으며 탐색이 부족하면 좋지 않은 기준 해 주변에 계획기가 갇힐 수 있다. 비용 균형이 적절하지 않으면 진동(Oscillation), 과도한 장애물 여유, 불충분한 경로 추종, 지나치게 보수적인 거동이 발생할 수 있다.

MPPI의 품질은 계산 지연시간(Computational Latency)에도 영향을 받는다. 모든 샘플링, 롤아웃 시뮬레이션, 비용 평가, 가중치 계산, 제어 갱신이 다음 명령이 필요한 시점 이전에 완료되어야 한다. 실제 로봇에서는 평균 실행 시간만으로 충분하지 않으며, 지연시간 변동(Latency Variation)과 최악 조건 동작(Worst-Case Behavior)도 중요하다. 명령이 지연되면 예측의 유효성이 감소하므로 효율적인 메모리 배치(Memory Layout), 병렬화(Parallelization), 예측 가능한 실행(Predictable Execution)이 중요한 구현 요소이다.

이동 로봇(Mobile Robot), 자율주행 차량(Autonomous Vehicle), 무인항공기(UAV), 기타 동역학적 제약을 가진 플랫폼에서 MPPI는 국소 궤적 생성(Local Trajectory Generation)과 제어를 위한 유연한 프레임워크를 제공한다. 핵심 순환 과정은 제어 섭동 샘플링, 미래 궤적 시뮬레이션, 비용 평가, 유망한 샘플의 가중치 계산, 기준 제어 입력 갱신, 첫 번째 제어 입력 실행, 그리고 반복으로 구성된다. 이러한 이동 시간 구간 과정을 통해 MPPI는 확률적 탐색(Stochastic Exploration), 최적 제어 추론(Optimal-Control Reasoning), 실시간 피드백 계획(Real-Time Feedback Planning)을 결합한다.

##  

## 04.06. Convex Decomposition IRIS for Collision Free Plan [w/Code]

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

IRIS, or Iterative Regional Inflation by Semidefinite Programming, is a method for constructing large convex regions of collision-free configuration or workspace. Instead of optimizing a complete trajectory directly through a highly nonconvex obstacle field, IRIS identifies convex subsets of free space that can later support efficient trajectory optimization. This converts difficult collision geometry into regions described by tractable convex constraints.

The fundamental challenge arises because obstacle-free space is generally nonconvex. Obstacles divide the environment into irregular regions, narrow passages, and disconnected components, making direct optimization difficult. Convex optimization becomes substantially easier when motion is restricted to convex subsets. IRIS therefore attempts to approximate useful portions of free space with large convex regions while ensuring that these regions do not intersect known obstacles.

A convex region has the important property that the line segment connecting any two points inside the region remains entirely inside it. For motion planning, this property simplifies many geometric constraints. If trajectory points are constrained to lie within an appropriate convex region, interpolation and optimization can be performed without repeatedly reasoning about arbitrary nonconvex obstacle boundaries, although robot geometry and dynamic feasibility may still require additional treatment.

IRIS begins from a seed point located in collision-free space. Around this seed, the algorithm constructs an initial ellipsoid representing a small candidate region. It then alternates between separating the ellipsoid from obstacles and enlarging the ellipsoid inside the resulting polytope. Through repeated iterations, the region expands until further improvement becomes sufficiently small or another termination criterion is satisfied.

Obstacle separation is performed by constructing hyperplanes between the current ellipsoid and surrounding obstacles. Each hyperplane places the ellipsoid on the free side while excluding the relevant obstacle from the candidate region. The intersection of these half-spaces forms a convex polytope. This polytope acts as a collision-free geometric container within which the ellipsoid can subsequently be enlarged.

For a polytope represented by linear inequalities \\(Ax\\leq b\\), each row of \\(A\\) and corresponding element of \\(b\\) define a half-space boundary. The complete intersection describes the current convex region. Linear inequality representations are particularly useful for trajectory optimization because state or configuration variables can be constrained directly using standard convex optimization machinery when the relevant motion variables are represented in the same space.

After constructing separating hyperplanes, IRIS computes a large ellipsoid contained within the polytope. The ellipsoid can be represented through a center and a positive-definite shape matrix. Maximizing its volume encourages the algorithm to expand into available free space rather than remaining near the original seed. The maximum-volume inscribed ellipsoid problem can be formulated using convex optimization and semidefinite programming concepts.

The algorithm then uses the enlarged ellipsoid to recompute obstacle-separating hyperplanes. Because the ellipsoid has expanded, the supporting hyperplanes can change and permit a larger polytope. A new maximum-volume ellipsoid is calculated inside that polytope, and the process repeats. This alternating structure explains the term iterative regional inflation: a small safe region is progressively inflated while obstacles constrain its growth.

The ellipsoid plays an important computational role beyond merely visualizing region size. By applying an affine transformation that maps the current ellipsoid to a unit sphere, obstacle separation can be expressed in a normalized coordinate system. Finding supporting hyperplanes relative to this transformed geometry provides a systematic mechanism for determining which obstacle surfaces limit further expansion of the convex region.

IRIS typically assumes that obstacles can be represented using convex geometry or decomposed into convex components. Complex nonconvex obstacles may therefore require geometric decomposition before region inflation. For each relevant convex obstacle, a separating plane is sought so that the candidate region remains on one side and the obstacle remains on the other. The resulting collection of planes defines a conservative collision-free approximation.

The generated region is conservative because IRIS does not attempt to reproduce the entire boundary of free space exactly. Instead, it finds a convex subset guaranteed or intended to remain within free space under the adopted geometric model. Some collision-free areas may be excluded from the region. This loss of coverage is accepted in exchange for obtaining simple convex constraints that are much easier to use during subsequent optimization.

A single convex region is often insufficient for a large environment containing corridors, rooms, turns, or multiple route alternatives. Multiple seed points can therefore be used to generate several IRIS regions. Their union provides a convex decomposition or convex cover of useful free space. Neighboring regions may intentionally overlap, allowing a planned trajectory to transition from one convex region to another.

These overlapping regions can be organized into a connectivity graph. Each node represents a collision-free convex set, while an edge indicates that two regions intersect or permit a feasible transition. Motion planning can then combine discrete search over the region graph with continuous optimization inside selected regions. This separates large-scale route selection from local continuous trajectory generation.

Convex-region planning is especially attractive when combined with trajectory optimization. Once a sequence of regions has been selected, trajectory states, control points, or polynomial segments can be constrained to remain within the associated polytopes. Collision avoidance that originally involved complex obstacle geometry can then be replaced, at least partly, by linear inequalities describing membership in precomputed collision-free regions.

For robotic manipulators, the relevant free space may be configuration space rather than ordinary Cartesian workspace. Obstacles in configuration space can have complicated nonlinear shapes because collision depends on the positions of multiple robot links. Extensions and related IRIS formulations can construct collision-free regions in configuration space, but computational difficulty increases substantially with dimensionality and geometric complexity.

The quality of an IRIS region depends strongly on seed placement. A seed in an open area may inflate into a large region, whereas a seed near a narrow passage or obstacle boundary may produce a small polytope. Multiple seeds, sampling strategies, or task-informed initialization can therefore improve coverage. Region generation may also be performed offline when the environment is largely static, reducing computation required during online planning.

Narrow passages remain challenging because a maximum-volume objective naturally favors expansion into broad open areas. A large region may therefore cover a room effectively while failing to extend through a thin corridor. Additional seeds placed inside narrow passages can address this problem. The desired decomposition is consequently influenced not only by geometry but also by the routes and tasks the robot is expected to execute.

Compared with CHOMP, TrajOpt, iLQR, and MPPI, IRIS serves a different role in optimization-based planning. Those methods primarily improve trajectories or control sequences, whereas IRIS constructs geometric regions in which subsequent optimization can be performed more safely and efficiently. It is therefore best viewed as a free-space preprocessing or decomposition technique that can complement trajectory optimizers rather than replace them.

IRIS is particularly valuable when the same environment is used for many planning queries. Collision-free convex regions can be generated once and reused for different start states, goals, and trajectory optimization problems. This amortizes the cost of geometric preprocessing. Warehouses, factories, structured manipulation workspaces, and other relatively stable environments are natural settings where reusable convex free-space representations can provide significant benefits.

Practical implementation must consider obstacle representation, numerical tolerances, region overlap, seed selection, dimensionality, and robot geometry. Conservative inflation improves safety margins but may reduce connectivity, while aggressive inflation can become sensitive to modeling or numerical errors. The resulting regions should therefore be validated against the collision model used by the actual planner and, where necessary, reduced to preserve operational clearance.

Within optimization-based robot planning, IRIS demonstrates how nonconvex collision avoidance can be reorganized before trajectory optimization begins. By alternating obstacle-separating hyperplanes with maximum-volume ellipsoid expansion, it transforms portions of complicated free space into convex polytopes. These regions can then support efficient continuous optimization, region-graph search, and repeated collision-free planning across complex robotic environments.

IRIS(Iterative Regional Inflation by Semidefinite Programming)는 충돌이 없는 구성 공간(Configuration Space) 또는 작업공간(Workspace)에서 큰 볼록 영역(Convex Region)을 구성하는 방법이다. 고도로 비볼록한 장애물 영역(Nonconvex Obstacle Field)에서 전체 궤적을 직접 최적화하는 대신, IRIS는 이후 효율적인 궤적 최적화(Trajectory Optimization)에 사용할 수 있는 자유 공간(Free Space)의 볼록 부분집합(Convex Subset)을 찾아낸다. 이를 통해 복잡한 충돌 기하학(Collision Geometry)을 다루기 쉬운 볼록 제약조건(Convex Constraint)으로 변환한다.

근본적인 어려움은 장애물이 없는 자유 공간이 일반적으로 비볼록(Nonconvex)이라는 점에서 발생한다. 장애물은 환경을 불규칙한 영역, 좁은 통로(Narrow Passage), 서로 분리된 구성 요소(Disconnected Component)로 나누기 때문에 직접적인 최적화가 어려워진다. 반면 움직임을 볼록 부분집합으로 제한하면 볼록 최적화(Convex Optimization)를 훨씬 쉽게 수행할 수 있다. 따라서 IRIS는 알려진 장애물과 교차하지 않으면서 자유 공간의 유용한 부분을 큰 볼록 영역으로 근사한다.

볼록 영역(Convex Region)은 영역 내부의 임의의 두 점을 연결하는 선분이 항상 해당 영역 내부에 존재한다는 중요한 특성을 가진다. 움직임 계획(Motion Planning)에서 이러한 특성은 많은 기하학적 제약조건을 단순화한다. 궤적 지점(Trajectory Point)을 적절한 볼록 영역 내부로 제한하면 임의의 비볼록 장애물 경계를 반복적으로 처리하지 않고도 보간(Interpolation)과 최적화를 수행할 수 있다. 다만 로봇 형상과 동역학적 실행 가능성(Dynamic Feasibility)은 추가적으로 고려해야 할 수 있다.

IRIS는 충돌이 없는 공간에 위치한 시드 포인트(Seed Point)에서 시작한다. 알고리즘은 이 시드 주변에 작은 후보 영역을 나타내는 초기 타원체(Ellipsoid)를 구성한다. 이후 타원체를 장애물로부터 분리하는 과정과 생성된 다면체(Polytope) 내부에서 타원체를 확장하는 과정을 번갈아 수행한다. 이러한 반복을 통해 추가적인 개선량이 충분히 작아지거나 다른 종료 기준(Termination Criterion)을 만족할 때까지 영역이 확장된다.

장애물 분리(Obstacle Separation)는 현재 타원체와 주변 장애물 사이에 초평면(Hyperplane)을 구성함으로써 수행된다. 각각의 초평면은 타원체를 자유 공간 쪽에 유지하면서 해당 장애물을 후보 영역에서 제외한다. 이러한 반공간(Half-Space)들의 교집합은 볼록 다면체(Convex Polytope)를 형성한다. 이 다면체는 충돌이 없는 기하학적 컨테이너(Geometric Container)의 역할을 하며, 그 내부에서 이후 타원체를 더욱 확장할 수 있다.

선형 부등식 \\(Ax\\leq b\\)로 표현되는 다면체에서 행렬 \\(A\\)의 각 행과 이에 대응하는 \\(b\\)의 원소는 하나의 반공간 경계(Half-Space Boundary)를 정의한다. 이러한 반공간의 전체 교집합이 현재의 볼록 영역을 나타낸다. 선형 부등식 표현(Linear Inequality Representation)은 관련 움직임 변수가 동일한 공간에서 표현될 경우 표준 볼록 최적화 기법을 사용하여 상태 또는 구성 변수를 직접 제한할 수 있기 때문에 궤적 최적화에 특히 유용하다.

분리 초평면(Separating Hyperplane)을 구성한 후 IRIS는 다면체 내부에 포함되는 큰 타원체를 계산한다. 타원체는 중심(Center)과 양의 정부호 형상 행렬(Positive-Definite Shape Matrix)을 이용하여 표현할 수 있다. 타원체의 부피를 최대화하면 알고리즘이 초기 시드 주변에 머무르지 않고 이용 가능한 자유 공간 방향으로 확장하도록 유도할 수 있다. 최대 부피 내접 타원체 문제(Maximum-Volume Inscribed Ellipsoid Problem)는 볼록 최적화와 반정의 계획법(Semidefinite Programming)의 개념을 이용하여 정식화할 수 있다.

이후 알고리즘은 확장된 타원체를 이용하여 장애물 분리 초평면을 다시 계산한다. 타원체가 확장되었기 때문에 지지 초평면(Supporting Hyperplane)이 변화할 수 있으며, 이를 통해 더 큰 다면체가 허용될 수 있다. 새로운 다면체 내부에서 다시 최대 부피 타원체를 계산하고 이러한 과정을 반복한다. 이와 같은 교대 구조(Alternating Structure)는 작은 안전 영역을 장애물이 허용하는 범위 내에서 점진적으로 팽창시킨다는 반복적 영역 팽창(Iterative Regional Inflation)의 의미를 나타낸다.

타원체는 단순히 영역의 크기를 시각화하는 것 이상의 중요한 계산적 역할을 수행한다. 현재 타원체를 단위 구(Unit Sphere)로 변환하는 아핀 변환(Affine Transformation)을 적용하면 장애물 분리 문제를 정규화된 좌표계(Normalized Coordinate System)에서 표현할 수 있다. 이렇게 변환된 기하학을 기준으로 지지 초평면을 탐색함으로써 어떤 장애물 표면이 볼록 영역의 추가적인 확장을 제한하는지를 체계적으로 결정할 수 있다.

IRIS는 일반적으로 장애물이 볼록 기하학(Convex Geometry)으로 표현되거나 여러 볼록 구성 요소(Convex Component)로 분해될 수 있다고 가정한다. 따라서 복잡한 비볼록 장애물은 영역 팽창 이전에 기하학적 분해(Geometric Decomposition)가 필요할 수 있다. 각각의 관련 볼록 장애물에 대해 후보 영역과 장애물이 서로 반대쪽에 위치하도록 분리 평면(Separating Plane)을 탐색한다. 이렇게 생성된 평면들의 집합이 보수적인 충돌 없는 근사 영역(Conservative Collision-Free Approximation)을 정의한다.

생성된 영역은 IRIS가 자유 공간의 전체 경계를 정확하게 재현하려 하지 않기 때문에 보수적(Conservative)이다. 대신 사용되는 기하학적 모델을 기준으로 자유 공간 내부에 유지되는 볼록 부분집합을 찾아낸다. 따라서 일부 충돌 없는 영역은 생성된 볼록 영역에서 제외될 수 있다. 이러한 공간 커버리지(Coverage)의 손실은 이후 최적화 과정에서 훨씬 쉽게 사용할 수 있는 단순한 볼록 제약조건을 얻기 위한 절충으로 받아들여진다.

복도, 방, 회전 구간 또는 여러 경로 대안(Route Alternative)이 존재하는 대규모 환경에서는 하나의 볼록 영역만으로 충분하지 않은 경우가 많다. 따라서 여러 개의 시드 포인트를 이용하여 복수의 IRIS 영역을 생성할 수 있다. 이러한 영역의 합집합(Union)은 유용한 자유 공간에 대한 볼록 분해(Convex Decomposition) 또는 볼록 커버(Convex Cover)를 제공한다. 인접 영역을 의도적으로 중첩(Overlap)시켜 계획된 궤적이 하나의 볼록 영역에서 다른 영역으로 전환될 수 있도록 할 수도 있다.

이러한 중첩 영역은 연결성 그래프(Connectivity Graph)로 구성할 수 있다. 각각의 노드(Node)는 충돌이 없는 볼록 집합을 나타내고, 에지(Edge)는 두 영역이 서로 교차하거나 실행 가능한 전환을 허용한다는 것을 의미한다. 움직임 계획은 영역 그래프에 대한 이산 탐색(Discrete Search)과 선택된 영역 내부의 연속 최적화(Continuous Optimization)를 결합할 수 있다. 이를 통해 대규모 경로 선택과 국소적인 연속 궤적 생성을 분리할 수 있다.

볼록 영역 계획(Convex-Region Planning)은 궤적 최적화와 결합할 때 특히 유용하다. 일련의 영역이 선택되면 궤적 상태(Trajectory State), 제어점(Control Point), 또는 다항식 구간(Polynomial Segment)을 해당 다면체 내부에 유지하도록 제한할 수 있다. 원래 복잡한 장애물 기하학을 포함하던 충돌 회피 문제를 적어도 부분적으로는 미리 계산된 충돌 없는 영역에 대한 소속 조건을 나타내는 선형 부등식으로 대체할 수 있다.

로봇 매니퓰레이터(Robot Manipulator)의 경우 관련 자유 공간은 일반적인 데카르트 작업공간(Cartesian Workspace)이 아니라 구성 공간(Configuration Space)이 될 수 있다. 여러 로봇 링크의 위치에 따라 충돌 여부가 결정되기 때문에 구성 공간의 장애물은 복잡한 비선형 형상을 가질 수 있다. IRIS의 확장 방법과 관련 정식화는 구성 공간에서 충돌 없는 영역을 생성할 수 있지만 차원(Dimensionality)과 기하학적 복잡성이 증가함에 따라 계산 난이도 역시 크게 증가한다.

IRIS 영역의 품질은 시드 배치(Seed Placement)에 크게 영향을 받는다. 개방된 공간의 시드는 큰 영역으로 팽창할 수 있지만 좁은 통로나 장애물 경계 근처의 시드는 작은 다면체를 생성할 수 있다. 따라서 여러 시드, 샘플링 전략(Sampling Strategy), 작업 정보를 활용한 초기화(Task-Informed Initialization)를 통해 공간 커버리지를 개선할 수 있다. 환경이 대부분 정적인 경우 영역 생성을 오프라인(Offline)으로 수행하여 온라인 계획(Online Planning)에 필요한 계산량을 줄일 수도 있다.

좁은 통로(Narrow Passage)는 최대 부피 목적함수(Maximum-Volume Objective)가 자연스럽게 넓은 개방 공간으로의 확장을 선호하기 때문에 여전히 어려운 문제이다. 따라서 하나의 큰 영역이 방과 같은 넓은 공간은 효과적으로 포함하면서도 좁은 복도까지 확장되지 못할 수 있다. 좁은 통로 내부에 추가 시드를 배치하면 이러한 문제를 해결할 수 있다. 따라서 필요한 분해 구조는 단순한 환경 기하학뿐만 아니라 로봇이 수행할 것으로 예상되는 경로와 작업에도 영향을 받는다.

CHOMP, TrajOpt, iLQR, MPPI와 비교하면 IRIS는 최적화 기반 계획(Optimization-Based Planning)에서 서로 다른 역할을 수행한다. 이러한 방법들이 주로 궤적 또는 제어 시퀀스(Control Sequence)를 개선하는 데 초점을 맞추는 반면, IRIS는 이후 최적화를 더욱 안전하고 효율적으로 수행할 수 있는 기하학적 영역(Geometric Region)을 구성한다. 따라서 IRIS는 궤적 최적화기를 대체하기보다는 이를 보완하는 자유 공간 전처리(Free-Space Preprocessing) 또는 분해 기법(Decomposition Technique)으로 이해하는 것이 적절하다.

IRIS는 동일한 환경에서 다수의 계획 질의(Planning Query)를 반복적으로 처리해야 할 때 특히 유용하다. 충돌 없는 볼록 영역을 한 번 생성한 후 서로 다른 시작 상태, 목표 상태, 궤적 최적화 문제에서 반복적으로 사용할 수 있다. 이를 통해 기하학적 전처리에 필요한 계산 비용을 여러 계획 문제에 분산(Amortization)할 수 있다. 창고(Warehouse), 공장(Factory), 구조화된 매니퓰레이션 작업공간(Structured Manipulation Workspace)과 같이 비교적 안정적인 환경은 재사용 가능한 볼록 자유 공간 표현의 이점을 활용하기에 적합하다.

실제 구현에서는 장애물 표현(Obstacle Representation), 수치 허용오차(Numerical Tolerance), 영역 중첩(Region Overlap), 시드 선택, 차원, 로봇 형상 등을 고려해야 한다. 보수적인 영역 팽창(Conservative Inflation)은 안전 여유를 향상시킬 수 있지만 영역 간 연결성을 감소시킬 수 있으며, 공격적인 팽창(Aggressive Inflation)은 모델링 또는 수치 오차에 민감해질 수 있다. 따라서 생성된 영역은 실제 계획기에서 사용하는 충돌 모델을 기준으로 검증하고 필요한 경우 운용 안전 여유(Operational Clearance)를 확보하도록 축소해야 한다.

최적화 기반 로봇 계획에서 IRIS는 궤적 최적화를 시작하기 전에 비볼록 충돌 회피(Nonconvex Collision Avoidance) 문제를 재구성할 수 있는 방법을 보여준다. 장애물 분리 초평면(Obstacle-Separating Hyperplane) 생성과 최대 부피 타원체(Maximum-Volume Ellipsoid) 확장을 번갈아 수행함으로써 복잡한 자유 공간의 일부를 볼록 다면체로 변환한다. 이렇게 생성된 영역은 복잡한 로봇 환경에서 효율적인 연속 최적화, 영역 그래프 탐색(Region-Graph Search), 반복적인 충돌 없는 계획(Collision-Free Planning)을 지원할 수 있다.

##  

## 04.07. Optimization Based Footstep Planning for Legged [w/Code]

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

Optimization-based footstep planning formulates locomotion for legged robots as a constrained optimization problem in which future foot placements, body motion, timing, and sometimes contact forces are selected simultaneously. Instead of choosing footsteps from a fixed discrete library, the planner searches continuous variables to find feasible contacts that advance the robot toward a goal while respecting terrain geometry, kinematics, stability, and dynamic limitations.

A footstep plan can be represented by a sequence of contact poses \\(p_0,p_1,\\ldots,p_N\\), where each pose may contain foot position, orientation, contact surface, and timing information. For humanoid or biped robots, the sequence alternates between left and right support contacts. Quadrupeds introduce several possible contact schedules, requiring the planner to determine or receive information about which feet remain in stance and which move during each phase.

The optimization objective describes desirable locomotion behavior. Typical terms penalize deviation from the target direction, excessive step length, large changes in orientation, unnecessary body motion, energy consumption, irregular timing, or proximity to unsafe terrain. Additional terms can encourage smooth progression, symmetric gait patterns, preferred foothold locations, and sufficient clearance from terrain boundaries. Weighting coefficients balance these competing locomotion objectives.

Footstep feasibility is strongly constrained by robot kinematics. A candidate foothold must remain within the reachable workspace of the corresponding leg, while joint limits and body geometry restrict relative foot positions. Excessively long, narrow, crossed, or rotated steps may be physically unreachable even when the terrain location itself is collision-free. Kinematic constraints therefore eliminate contact configurations that cannot be realized by the robot mechanism.

Terrain geometry introduces another set of constraints. Candidate contacts should lie on surfaces capable of supporting the foot and should provide sufficient contact area. Surface position, orientation, slope, roughness, and boundary distance can influence whether a foothold is acceptable. Perception systems may represent terrain using elevation maps, point clouds, meshes, planar regions, signed-distance fields, or segmented contact surfaces from which feasible foothold regions are derived.

Convex terrain regions are particularly useful for optimization-based planning. A safe contact patch can be approximated by a convex polygon or polytope, allowing foot-position variables to be constrained through linear inequalities. Complex terrain can be decomposed into several candidate support regions. The planner then selects appropriate regions and optimizes continuous foot positions inside them, combining discrete terrain choices with continuous optimization.

This combination naturally produces mixed discrete-continuous optimization. Discrete variables may identify the selected support surface, contact sequence, gait mode, or left-right stepping decision, while continuous variables describe exact foot poses and timing. Mixed-Integer Programming can express such decisions explicitly, although computational cost can increase rapidly with the number of candidate regions, footsteps, and logical alternatives considered.

Sequential convex optimization provides another practical approach. Nonlinear reachability, collision, stability, and terrain constraints can be locally approximated around an initial footstep sequence. A convex subproblem is solved to improve the current plan, after which constraints are recomputed and the procedure repeats. Trust regions can limit each update so that local approximations remain valid while the footsteps progressively move toward feasible and lower-cost configurations.

Static stability can be expressed using the robot\'s center of mass and support polygon. During quasi-static locomotion, the projected center of mass should remain within an appropriate support region generated by active contacts. This concept is intuitive and computationally convenient, but it becomes insufficient for faster locomotion because momentum and inertial effects become significant. Dynamic planning therefore requires richer stability models.

The Zero Moment Point, or ZMP, provides a classical dynamic stability criterion for legged locomotion. Under suitable modeling assumptions, the planner constrains the ZMP to remain inside the support polygon. Simplified models such as the Linear Inverted Pendulum Model can relate center-of-mass motion, ZMP location, and foot placement through equations suitable for optimization. These models enable efficient prediction of dynamically meaningful walking behavior.

More general planners may optimize centroidal dynamics, including center-of-mass acceleration, linear momentum, angular momentum, and contact forces. Contact forces must satisfy force balance and friction constraints while preventing physically impossible pulling forces at unilateral contacts. Friction cones are often approximated using linear pyramids or other convex forms so that contact feasibility can be incorporated efficiently into numerical optimization.

Footstep timing is closely coupled with spatial placement. A foothold that is dynamically feasible with a long step duration may become infeasible when the same step must be executed rapidly. Optimization variables can therefore include contact duration, swing duration, or phase transition times. Jointly optimizing where and when the robot steps allows the planner to trade locomotion speed against stability, reachability, control effort, and terrain difficulty.

Swing-foot motion must also be considered because feasible contact poses alone do not guarantee feasible transitions. The foot must travel from its current contact to the next foothold without colliding with terrain or other robot links. Swing trajectories may be optimized directly or generated after footstep planning using clearance constraints. Uneven terrain, stairs, and isolated stepping stones make swing-foot clearance particularly important.

Collision avoidance extends beyond the swing foot. Knees, legs, torso, and other body components may collide with terrain or environmental structures even when every planned foothold is valid. Full-body collision models can be expensive, so planners frequently use simplified geometric approximations during footstep optimization and perform more detailed validation afterward. Conservative margins can reduce the risk introduced by these approximations.

Initial footstep sequences strongly affect optimization because legged locomotion problems are generally nonconvex. A nominal walking pattern, heuristic foothold sequence, lattice search, graph-based planner, or previous solution can provide initialization. Optimization then adjusts positions, orientations, timing, or body states. A poor initial sequence may place the solver in an infeasible region or an undesirable local minimum from which local optimization cannot recover.

A useful planning architecture therefore separates global route reasoning from local footstep refinement. A global planner identifies the general direction or traversable terrain corridor, while an optimization-based footstep planner selects precise contacts inside that corridor. Whole-body trajectory optimization or model predictive control can subsequently generate dynamically consistent body and joint motions that realize the selected contact sequence.

Receding-horizon footstep planning improves adaptability in uncertain environments. Rather than optimizing an entire long route once, the robot repeatedly plans several future contacts using its latest state and terrain estimate. After executing one or more steps, perception updates the terrain representation and optimization is performed again. This allows the robot to respond to localization errors, imperfect terrain models, disturbances, and newly observed obstacles.

Robustness is particularly important because planned footholds are executed using uncertain perception and control. Terrain height, surface orientation, friction, and robot state estimates may differ from their nominal values. Safety margins can shrink allowable contact regions or increase distance from edges. More advanced formulations may explicitly optimize against uncertainty, chance constraints, disturbance sets, or multiple terrain hypotheses to reduce sensitivity to modeling errors.

Different legged platforms impose different planning structures. Bipeds require careful balance during alternating single- and double-support phases, while quadrupeds can exploit additional contacts and diverse gait patterns. Humanoids may also use hands or other body contacts, turning footstep planning into general multi-contact planning. As contact possibilities increase, the optimization problem becomes richer but also substantially more computationally demanding.

Optimization-based footstep planning is particularly valuable on stairs, stepping stones, rubble, slopes, and discontinuous terrain where regular periodic stepping is inadequate. The planner can place each foot according to local geometry while considering future reachability and stability. This ability distinguishes terrain-aware legged navigation from simple gait generation, which may assume relatively uniform ground and predetermined step patterns.

Real-time performance depends on horizon length, terrain representation, number of candidate contacts, model complexity, solver choice, and initialization quality. Simplified dynamics and convex approximations improve speed, while full-body dynamics and large mixed-integer formulations increase computational demand. Practical systems therefore use hierarchical planning, warm starts, parallel evaluation, reduced-order models, and limited horizons to obtain sufficiently fast solutions.

The resulting footstep plan must be validated before execution and coordinated with lower-level locomotion control. Reachability, collision clearance, contact stability, friction, timing, and dynamic consistency should be checked using the robot model. The controller must then track body and swing-foot trajectories while compensating for disturbances. Planning and control therefore form a tightly coupled pipeline rather than independent stages.

Within optimization-based motion planning, footstep planning demonstrates how trajectory optimization principles extend to hybrid contact systems. Continuous body motion interacts with discrete contact transitions, terrain geometry, dynamics, and stability constraints. By optimizing contact locations, timing, and locomotion objectives together, the approach provides a systematic framework for generating feasible legged motion across complex environments.

최적화 기반 발걸음 계획(Optimization-Based Footstep Planning)은 다족 로봇(Legged Robot)의 보행을 제약 최적화 문제(Constrained Optimization Problem)로 정식화하여 미래의 발 배치(Foot Placement), 몸체 움직임(Body Motion), 타이밍(Timing), 그리고 경우에 따라 접촉력(Contact Force)을 동시에 결정한다. 고정된 이산 발걸음 라이브러리(Discrete Footstep Library)에서 발걸음을 선택하는 대신, 계획기는 연속 변수(Continuous Variable)를 탐색하여 지형 형상, 운동학, 안정성, 동역학적 한계를 만족하면서 로봇을 목표 방향으로 이동시키는 실행 가능한 접촉(Feasible Contact)을 찾는다.

발걸음 계획(Footstep Plan)은 접촉 자세(Contact Pose)의 시퀀스 \\(p_0,p_1,\\ldots,p_N\\)로 표현할 수 있으며, 각 자세에는 발의 위치, 방향, 접촉 표면(Contact Surface), 타이밍 정보가 포함될 수 있다. 휴머노이드(Humanoid) 또는 이족 로봇(Biped Robot)의 경우 시퀀스는 왼발과 오른발의 지지 접촉을 번갈아 구성한다. 사족 로봇(Quadruped)은 여러 접촉 스케줄(Contact Schedule)을 가질 수 있으므로 각 단계에서 어떤 발이 지지 상태(Stance)를 유지하고 어떤 발이 이동하는지를 계획기가 결정하거나 외부에서 제공받아야 한다.

최적화 목적함수(Optimization Objective)는 바람직한 보행 동작(Locomotion Behavior)을 정의한다. 일반적인 비용 항은 목표 방향으로부터의 편차, 과도한 보폭(Step Length), 큰 방향 변화, 불필요한 몸체 움직임, 에너지 소비, 불규칙한 타이밍, 위험한 지형에 대한 근접도를 억제한다. 추가 비용 항을 통해 부드러운 진행, 대칭적인 보행 패턴(Symmetric Gait Pattern), 선호 발 디딤 위치(Preferred Foothold), 지형 경계로부터 충분한 여유 거리를 유도할 수 있다. 가중 계수(Weighting Coefficient)는 이러한 상충하는 보행 목적 사이의 균형을 조절한다.

발걸음의 실행 가능성(Footstep Feasibility)은 로봇 운동학(Robot Kinematics)에 의해 강하게 제한된다. 후보 발 디딤점(Candidate Foothold)은 해당 다리의 도달 가능 작업공간(Reachable Workspace) 안에 있어야 하며, 관절 한계(Joint Limit)와 몸체 형상(Body Geometry)은 발 사이의 상대적 위치를 제한한다. 지나치게 길거나 좁은 보폭, 다리가 서로 교차하는 발걸음, 또는 과도하게 회전된 발걸음은 지형 자체에 충돌이 없더라도 물리적으로 도달할 수 없을 수 있다. 따라서 운동학적 제약조건(Kinematic Constraint)은 로봇 메커니즘으로 실현할 수 없는 접촉 구성을 제거한다.

지형 형상(Terrain Geometry)은 또 다른 제약조건을 제공한다. 후보 접촉점은 발을 지지할 수 있는 표면 위에 존재해야 하며 충분한 접촉 면적(Contact Area)을 제공해야 한다. 표면의 위치, 방향, 경사(Slope), 거칠기(Roughness), 경계까지의 거리는 해당 발 디딤점의 적합성에 영향을 줄 수 있다. 인지 시스템(Perception System)은 고도 지도(Elevation Map), 포인트 클라우드(Point Cloud), 메시(Mesh), 평면 영역(Planar Region), 부호 거리장(Signed-Distance Field), 또는 분할된 접촉 표면(Segmented Contact Surface)을 이용하여 지형을 표현하고 실행 가능한 발 디딤 영역을 생성할 수 있다.

볼록 지형 영역(Convex Terrain Region)은 최적화 기반 계획에서 특히 유용하다. 안전한 접촉 패치(Contact Patch)를 볼록 다각형(Convex Polygon)이나 볼록 다면체(Convex Polytope)로 근사하면 발 위치 변수를 선형 부등식(Linear Inequality)으로 제한할 수 있다. 복잡한 지형은 여러 후보 지지 영역(Candidate Support Region)으로 분해할 수 있다. 계획기는 적절한 영역을 선택하고 그 내부에서 연속적인 발 위치를 최적화함으로써 이산적인 지형 선택과 연속 최적화(Continuous Optimization)를 결합한다.

이러한 결합은 자연스럽게 혼합 이산-연속 최적화(Mixed Discrete-Continuous Optimization) 문제를 형성한다. 이산 변수(Discrete Variable)는 선택된 지지 표면, 접촉 시퀀스, 보행 모드(Gait Mode), 좌우 발걸음 결정 등을 나타낼 수 있으며, 연속 변수는 정확한 발 자세와 타이밍을 표현한다. 혼합정수계획법(Mixed-Integer Programming)을 이용하면 이러한 결정을 명시적으로 표현할 수 있지만 후보 영역, 발걸음 수, 논리적 대안(Logical Alternative)의 수가 증가하면 계산 비용이 빠르게 증가할 수 있다.

순차 볼록 최적화(Sequential Convex Optimization)는 또 다른 실용적인 접근법을 제공한다. 비선형 도달 가능성(Nonlinear Reachability), 충돌, 안정성, 지형 제약조건을 초기 발걸음 시퀀스 주변에서 국소적으로 근사할 수 있다. 볼록 하위 문제(Convex Subproblem)를 해결하여 현재 계획을 개선한 뒤 제약조건을 다시 계산하고 이 과정을 반복한다. 신뢰 영역(Trust Region)은 각 갱신의 크기를 제한하여 국소 근사의 유효성을 유지하면서 발걸음이 점진적으로 실행 가능하고 비용이 낮은 구성으로 이동하도록 한다.

정적 안정성(Static Stability)은 로봇의 질량중심(Center of Mass)과 지지 다각형(Support Polygon)을 이용하여 표현할 수 있다. 준정적 보행(Quasi-Static Locomotion)에서는 투영된 질량중심이 활성 접촉(Active Contact)에 의해 생성되는 적절한 지지 영역 내부에 유지되어야 한다. 이 개념은 직관적이고 계산하기 편리하지만 빠른 보행에서는 운동량(Momentum)과 관성 효과(Inertial Effect)가 중요해지므로 충분하지 않다. 따라서 동적 계획(Dynamic Planning)에는 더욱 정교한 안정성 모델이 필요하다.

영 모멘트 점(Zero Moment Point, ZMP)은 다족 로봇 보행에서 사용되는 전통적인 동적 안정성 기준(Dynamic Stability Criterion)을 제공한다. 적절한 모델링 가정하에서 계획기는 ZMP가 지지 다각형 내부에 유지되도록 제약할 수 있다. 선형 역진자 모델(Linear Inverted Pendulum Model, LIPM)과 같은 단순화된 모델은 질량중심 움직임, ZMP 위치, 발 배치 사이의 관계를 최적화에 적합한 방정식으로 표현할 수 있다. 이를 통해 동역학적으로 의미 있는 보행 동작을 효율적으로 예측할 수 있다.

보다 일반적인 계획기는 질량중심 가속도, 선형 운동량(Linear Momentum), 각운동량(Angular Momentum), 접촉력을 포함하는 중심 동역학(Centroidal Dynamics)을 최적화할 수 있다. 접촉력은 힘의 평형(Force Balance)과 마찰 제약조건(Friction Constraint)을 만족하면서 단방향 접촉(Unilateral Contact)에서 물리적으로 불가능한 당기는 힘이 발생하지 않도록 해야 한다. 마찰 원뿔(Friction Cone)은 선형 피라미드(Linear Pyramid) 또는 기타 볼록 형태로 근사하여 접촉 실행 가능성을 수치 최적화에 효율적으로 포함할 수 있다.

발걸음 타이밍(Footstep Timing)은 공간적 발 배치와 밀접하게 결합되어 있다. 긴 보행 시간이 주어지면 동역학적으로 실행 가능한 발 디딤점도 동일한 발걸음을 빠르게 수행해야 하는 경우 실행 불가능해질 수 있다. 따라서 최적화 변수에는 접촉 지속시간(Contact Duration), 스윙 지속시간(Swing Duration), 또는 위상 전환 시간(Phase Transition Time)을 포함할 수 있다. 로봇이 어디에 그리고 언제 발을 디딜지를 함께 최적화하면 보행 속도와 안정성, 도달 가능성, 제어 노력, 지형 난이도 사이의 균형을 조절할 수 있다.

스윙 풋 움직임(Swing-Foot Motion)도 고려해야 한다. 실행 가능한 접촉 자세만 확보했다고 해서 접촉 사이의 전환까지 실행 가능하다는 의미는 아니기 때문이다. 발은 현재 접촉 위치에서 다음 발 디딤점까지 이동하는 동안 지형이나 다른 로봇 링크와 충돌하지 않아야 한다. 스윙 궤적(Swing Trajectory)은 직접 최적화하거나 발걸음 계획 이후 여유 높이 제약조건(Clearance Constraint)을 이용하여 생성할 수 있다. 불규칙 지형, 계단, 고립된 디딤돌(Stepping Stone)에서는 스윙 풋의 지형 여유가 특히 중요하다.

충돌 회피(Collision Avoidance)는 스윙 풋에만 국한되지 않는다. 계획된 모든 발 디딤점이 유효하더라도 무릎, 다리, 몸통 및 기타 신체 구성 요소가 지형이나 주변 구조물과 충돌할 수 있다. 전신 충돌 모델(Full-Body Collision Model)은 계산 비용이 높을 수 있으므로 발걸음 최적화에서는 단순화된 기하학적 근사(Geometric Approximation)를 사용하고 이후 더욱 상세한 검증을 수행하는 경우가 많다. 보수적인 안전 여유(Conservative Margin)를 적용하면 이러한 근사로 인한 위험을 줄일 수 있다.

다족 보행 문제는 일반적으로 비볼록(Nonconvex)이므로 초기 발걸음 시퀀스(Initial Footstep Sequence)가 최적화에 큰 영향을 준다. 기준 보행 패턴(Nominal Walking Pattern), 휴리스틱 발 디딤 시퀀스, 격자 탐색(Lattice Search), 그래프 기반 계획기(Graph-Based Planner), 또는 이전 해를 초기값으로 사용할 수 있다. 이후 최적화를 통해 위치, 방향, 타이밍, 몸체 상태를 조정한다. 부적절한 초기 시퀀스는 최적화기를 실행 불가능 영역이나 국소 최솟값(Local Minimum)에 위치시켜 국소 최적화만으로 회복하기 어렵게 만들 수 있다.

따라서 유용한 계획 아키텍처는 전역 경로 추론(Global Route Reasoning)과 국소 발걸음 정제(Local Footstep Refinement)를 분리한다. 전역 계획기(Global Planner)는 전체적인 진행 방향이나 이동 가능한 지형 회랑(Traversable Terrain Corridor)을 결정하고, 최적화 기반 발걸음 계획기는 그 회랑 내부에서 정확한 접촉점을 선택한다. 이후 전신 궤적 최적화(Whole-Body Trajectory Optimization) 또는 모델 예측 제어(Model Predictive Control, MPC)를 통해 선택된 접촉 시퀀스를 실현하는 동역학적으로 일관된 몸체 및 관절 움직임을 생성할 수 있다.

이동 시간 구간 발걸음 계획(Receding-Horizon Footstep Planning)은 불확실한 환경에서 적응성을 향상시킨다. 전체 장거리 경로를 한 번에 최적화하는 대신 로봇의 최신 상태와 지형 추정값을 이용하여 앞으로의 몇 개 접촉을 반복적으로 계획한다. 하나 이상의 발걸음을 실행한 후 인지 시스템이 지형 표현을 갱신하고 다시 최적화를 수행한다. 이를 통해 위치추정 오차, 불완전한 지형 모델, 외란, 새롭게 관측된 장애물에 대응할 수 있다.

계획된 발 디딤점은 불확실한 인지 및 제어 정보를 기반으로 실행되므로 강건성(Robustness)이 특히 중요하다. 지형 높이, 표면 방향, 마찰(Friction), 로봇 상태 추정값은 기준값과 다를 수 있다. 안전 여유를 적용하여 허용 가능한 접촉 영역을 축소하거나 가장자리로부터의 거리를 증가시킬 수 있다. 보다 발전된 정식화에서는 불확실성(Uncertainty), 확률 제약조건(Chance Constraint), 외란 집합(Disturbance Set), 복수의 지형 가설(Terrain Hypothesis)을 명시적으로 최적화하여 모델링 오차에 대한 민감도를 낮출 수 있다.

서로 다른 다족 플랫폼(Legged Platform)은 서로 다른 계획 구조를 요구한다. 이족 로봇은 단일 지지 단계(Single-Support Phase)와 이중 지지 단계(Double-Support Phase)를 번갈아 수행하면서 세심한 균형 제어가 필요하며, 사족 로봇은 더 많은 접촉점과 다양한 보행 패턴을 활용할 수 있다. 휴머노이드는 손이나 기타 신체 접촉까지 사용할 수 있어 발걸음 계획이 일반적인 다중 접촉 계획(Multi-Contact Planning)으로 확장된다. 접촉 가능성이 증가할수록 최적화 문제는 더욱 풍부해지지만 계산 요구량도 크게 증가한다.

최적화 기반 발걸음 계획은 규칙적인 주기적 보행만으로 대응하기 어려운 계단(Stairs), 디딤돌, 잔해(Rubble), 경사면(Slope), 불연속 지형(Discontinuous Terrain)에서 특히 유용하다. 계획기는 미래의 도달 가능성과 안정성을 고려하면서 국소 지형 형상에 따라 각각의 발을 배치할 수 있다. 이러한 능력은 비교적 균일한 지면과 미리 정해진 보폭 패턴을 가정하는 단순 보행 생성(Gait Generation)과 지형 인식 다족 내비게이션(Terrain-Aware Legged Navigation)을 구분하는 중요한 특징이다.

실시간 성능(Real-Time Performance)은 계획 시간 구간(Horizon Length), 지형 표현, 후보 접촉 수, 모델 복잡도, 최적화기 선택(Solver Choice), 초기화 품질에 영향을 받는다. 단순화된 동역학과 볼록 근사는 계산 속도를 높이는 반면 전신 동역학(Full-Body Dynamics)과 대규모 혼합정수 정식화(Large Mixed-Integer Formulation)는 계산량을 증가시킨다. 따라서 실제 시스템은 계층적 계획(Hierarchical Planning), 웜 스타트(Warm Start), 병렬 평가(Parallel Evaluation), 축소 차수 모델(Reduced-Order Model), 제한된 계획 구간을 이용하여 충분히 빠른 해를 얻는다.

생성된 발걸음 계획은 실제 실행 전에 검증하고 하위 수준 보행 제어(Low-Level Locomotion Control)와 연계해야 한다. 로봇 모델을 이용하여 도달 가능성, 충돌 여유(Collision Clearance), 접촉 안정성, 마찰, 타이밍, 동역학적 일관성(Dynamic Consistency)을 확인해야 한다. 이후 제어기는 외란을 보상하면서 몸체 및 스윙 풋 궤적을 추종해야 한다. 따라서 계획과 제어는 서로 독립적인 단계가 아니라 긴밀하게 결합된 파이프라인(Coupled Pipeline)을 형성한다.

최적화 기반 움직임 계획(Optimization-Based Motion Planning)에서 발걸음 계획은 궤적 최적화 원리가 하이브리드 접촉 시스템(Hybrid Contact System)으로 어떻게 확장될 수 있는지를 보여준다. 연속적인 몸체 움직임은 이산적인 접촉 전환(Discrete Contact Transition), 지형 형상, 동역학, 안정성 제약조건과 상호작용한다. 접촉 위치, 타이밍, 보행 목적을 함께 최적화함으로써 복잡한 환경에서 실행 가능한 다족 로봇 움직임을 생성하기 위한 체계적인 프레임워크를 제공한다.

##  

## 04.08. Whole Body Motion Optimization for Humanoid [w/Code]

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

Whole-body motion optimization for humanoid robots formulates coordinated movement of the entire robot as a unified optimization problem. Rather than planning the legs, arms, torso, and center of mass independently, the method simultaneously considers their coupled motion. This is essential because changing one body segment affects balance, reachability, collision risk, momentum, and contact forces throughout the humanoid system.

A humanoid configuration is commonly represented by a floating-base state together with many actuated joint variables. The floating base describes the position and orientation of the pelvis or torso relative to the world, while joint coordinates represent the legs, arms, torso, neck, and other mechanisms. Optimization may additionally include velocities, accelerations, torques, contact forces, and contact timing depending on the required physical fidelity.

The trajectory can be discretized into states \\(x_0,x_1,\\ldots,x_T\\) and controls \\(u_0,u_1,\\ldots,u_{T-1}\\). These variables describe how the complete humanoid evolves through time. The optimization searches over the trajectory while satisfying boundary conditions, task objectives, robot dynamics, contact constraints, and environmental restrictions. Increasing the number of time steps improves temporal resolution but also increases computational complexity.

Whole-body planning is fundamentally a multi-objective problem. A humanoid may need to move toward a target, maintain balance, place its feet accurately, reach with one hand, orient its torso, avoid obstacles, and minimize unnecessary joint motion simultaneously. These objectives are represented through weighted costs or hierarchical tasks so that the optimizer can coordinate competing requirements within a single motion-generation framework.

Task-space objectives are often defined using forward kinematics. A hand may be required to reach a desired Cartesian pose, a foot may need to remain fixed on a support surface, or the torso may be encouraged to maintain a preferred orientation. Kinematic Jacobians relate changes in joint variables to task-space motion, enabling optimization algorithms to determine how coordinated whole-body adjustments can reduce task errors.

Kinematic feasibility imposes joint position, velocity, and sometimes acceleration limits throughout the motion. The optimizer must prevent configurations that exceed mechanical ranges or require unrealistic rates of movement. Additional posture costs can keep joints away from singularities and extreme configurations. Redundant degrees of freedom allow multiple joint arrangements to accomplish the same task, giving optimization freedom to satisfy secondary objectives.

Balance is one of the defining constraints of humanoid motion. During slow quasi-static movement, the projected center of mass can be required to remain inside the support polygon formed by active contacts. For more dynamic motion, this simple criterion becomes insufficient because acceleration and momentum influence stability. Whole-body optimization therefore often incorporates dynamic quantities such as ZMP, centroidal momentum, or contact wrench constraints.

Centroidal dynamics provide a reduced representation connecting whole-body motion to external contact forces. The robot\'s center-of-mass acceleration depends on gravity and the forces applied through the feet, hands, or other contacts, while changes in angular momentum depend on the associated moments. Optimizing these quantities allows the planner to reason about balance without always requiring full rigid-body dynamics at every stage.

Higher-fidelity formulations directly impose multibody equations of motion. These equations relate generalized positions, velocities, accelerations, actuator torques, and contact forces through the robot mass matrix, Coriolis and centrifugal effects, gravity, and contact Jacobians. Such formulations can generate dynamically consistent trajectories but introduce large nonlinear optimization problems, particularly for humanoids containing many actuated degrees of freedom.

Contacts introduce both continuous and discrete decisions. When a foot is in stance, its position should remain consistent with the supporting surface and appropriate contact forces must exist. During swing, that contact disappears and the foot must move toward another support region. Walking, climbing, and multi-contact manipulation therefore create hybrid systems in which continuous dynamics interact with discrete changes in contact mode.

Contact forces must satisfy physical feasibility conditions. A unilateral ground contact can push the robot but cannot pull it toward the surface. Tangential forces must remain within friction limits to avoid slipping, while center-of-pressure or contact-wrench constraints may restrict how loads are distributed across the foot. Friction cones are frequently approximated by convex pyramids to make these constraints easier to optimize.

Footstep placement and whole-body motion are tightly coupled. Moving a foot changes the future support geometry, while the center of mass and torso must move appropriately before and during the step. A hierarchical architecture may first generate approximate footsteps and then optimize the complete body trajectory. More integrated approaches optimize contacts, body states, and forces together, potentially producing better motions at substantially greater computational cost.

Humanoid arms are not merely manipulation devices; they can contribute significantly to whole-body balance and momentum regulation. Arm motion can compensate for torso rotation, reduce angular momentum, improve reachability, or prepare for contact with the environment. Whole-body optimization can exploit this redundancy automatically, coordinating arm, torso, pelvis, and leg motion rather than treating upper-body movement as an independent secondary animation.

Collision avoidance must account for both the environment and the robot itself. Legs may collide during stepping, arms may intersect the torso, and the body may strike nearby structures while reaching or turning. Collision geometry can be represented by capsules, spheres, convex primitives, meshes, or signed-distance models. Differentiable distance information is particularly useful because it provides gradients that push optimized configurations away from collision.

Environmental contact can also be intentionally exploited. A humanoid may place a hand on a wall, railing, ladder, or support structure to improve stability or enable movement that would otherwise be infeasible. This extends ordinary footstep planning into multi-contact whole-body planning. The optimizer must then determine feasible body configurations and force distributions across several contacts while maintaining friction and reachability constraints.

Whole-body motion optimization is strongly nonconvex. Forward kinematics, rotations, collision avoidance, contact switching, friction, and nonlinear dynamics create multiple local solutions and infeasible regions. Initialization therefore has major influence on convergence. Initial trajectories may come from inverse kinematics, footstep planners, motion libraries, simplified centroidal models, previous solutions, or manually constructed reference motions.

Sequential quadratic programming, sequential convex optimization, direct collocation, differential dynamic programming, and related nonlinear optimization methods can be applied depending on the formulation. Direct transcription methods treat states and controls across time as optimization variables, while shooting methods generate states by integrating dynamics from controls. Each representation creates different tradeoffs among sparsity, constraint handling, numerical conditioning, and computational efficiency.

Direct collocation is particularly useful when dynamics and constraints must be enforced throughout a trajectory. States and controls are defined at discrete knot points, and additional equations ensure consistency with the dynamics between those points. This converts continuous-time optimal control into a structured nonlinear program. Sparse solvers can exploit the fact that each time step interacts primarily with neighboring trajectory states.

Hierarchical optimization can preserve task priorities when simple weighted sums are inappropriate. Maintaining contact or avoiding falls may receive higher priority than tracking a preferred posture, while hand accuracy may be more important than minimizing small joint motions. Lexicographic or hierarchical formulations prevent lower-priority objectives from degrading critical constraints, although they can require more sophisticated numerical methods.

Trajectory duration and contact timing can also become optimization variables. Slower movement may reduce required forces and improve stability, while faster motion may require greater momentum and actuator effort. Jointly optimizing spatial motion and timing allows the system to discover motions that would be impossible under a fixed schedule. However, variable timing introduces additional nonlinear coupling between dynamics, contacts, and trajectory discretization.

Uncertainty complicates execution because optimized trajectories rely on estimated robot states, terrain geometry, friction, and dynamic models. Robust formulations can introduce safety margins, uncertainty sets, or risk-sensitive costs. Receding-horizon optimization provides another mechanism: only an initial portion of the planned motion is executed before the state is measured again and the remaining trajectory is recomputed using updated information.

Model Predictive Control can therefore extend whole-body optimization from offline planning toward feedback control. At each cycle, a finite-horizon whole-body problem is solved using the current robot state and contact conditions. The first control action is executed and the horizon shifts forward. Warm starts, reduced-order models, sparse computation, and parallel hardware are important for achieving control rates suitable for practical humanoid systems.

A common practical architecture distributes complexity across several levels. A global planner determines where the humanoid should travel, a footstep or contact planner determines useful support locations, a centroidal planner determines approximate body and momentum evolution, and whole-body optimization generates consistent joint-level motion. Lower-level controllers then track the resulting references while responding rapidly to disturbances and modeling errors.

Validation remains essential even when numerical optimization converges successfully. The resulting motion should be checked for collisions, joint limits, actuator limits, friction feasibility, contact stability, dynamic consistency, and sufficient environmental clearance. Simulation can reveal sensitivity to state-estimation error or contact uncertainty before execution. Numerical convergence alone does not guarantee that a motion is physically safe or robust.

Whole-body motion optimization ultimately treats the humanoid as one coupled physical system rather than a collection of independently controlled limbs. By jointly reasoning about kinematics, dynamics, balance, contacts, collision avoidance, task objectives, forces, and timing, it provides a systematic foundation for walking, reaching, manipulation, climbing, and multi-contact behavior in complex environments.

휴머노이드 로봇(Humanoid Robot)을 위한 전신 움직임 최적화(Whole-Body Motion Optimization)는 로봇 전체의 협조된 움직임(Coordinated Movement)을 하나의 통합 최적화 문제(Unified Optimization Problem)로 정식화한다. 다리, 팔, 몸통, 질량중심(Center of Mass)을 독립적으로 계획하는 대신 이들의 결합된 움직임(Coupled Motion)을 동시에 고려한다. 하나의 신체 부위 변화가 휴머노이드 전체의 균형, 도달 가능성, 충돌 위험, 운동량(Momentum), 접촉력(Contact Force)에 영향을 미치므로 이러한 통합적 접근이 필수적이다.

휴머노이드 구성(Humanoid Configuration)은 일반적으로 부유 기저 상태(Floating-Base State)와 다수의 구동 관절 변수(Actuated Joint Variable)를 함께 사용하여 표현한다. 부유 기저는 세계 좌표계(World Frame)에 대한 골반 또는 몸통의 위치와 방향을 나타내며, 관절 좌표(Joint Coordinate)는 다리, 팔, 몸통, 목 및 기타 메커니즘을 표현한다. 요구되는 물리적 충실도(Physical Fidelity)에 따라 최적화 변수에 속도, 가속도, 토크, 접촉력, 접촉 타이밍(Contact Timing)을 추가할 수 있다.

궤적(Trajectory)은 상태 \\(x_0,x_1,\\ldots,x_T\\)와 제어 입력 \\(u_0,u_1,\\ldots,u_{T-1}\\)로 이산화할 수 있다. 이러한 변수는 시간에 따라 전체 휴머노이드가 어떻게 변화하는지를 나타낸다. 최적화는 경계조건(Boundary Condition), 작업 목표(Task Objective), 로봇 동역학(Robot Dynamics), 접촉 제약조건(Contact Constraint), 환경 제약조건(Environmental Restriction)을 만족하면서 전체 궤적을 탐색한다. 시간 단계 수를 늘리면 시간 해상도(Temporal Resolution)는 향상되지만 계산 복잡도도 증가한다.

전신 계획(Whole-Body Planning)은 근본적으로 다목적 문제(Multi-Objective Problem)이다. 휴머노이드는 목표 방향으로 이동하면서 균형을 유지하고, 발을 정확하게 배치하며, 한 손으로 목표에 도달하고, 몸통 자세를 조정하며, 장애물을 회피하고, 불필요한 관절 움직임을 최소화해야 할 수 있다. 이러한 목표는 가중 비용(Weighted Cost)이나 계층적 작업(Hierarchical Task)으로 표현되어 하나의 움직임 생성 프레임워크에서 서로 경쟁하는 요구조건을 조정할 수 있도록 한다.

작업공간 목적(Task-Space Objective)은 흔히 순기구학(Forward Kinematics)을 이용하여 정의된다. 손은 원하는 데카르트 자세(Cartesian Pose)에 도달해야 할 수 있고, 발은 지지 표면(Support Surface)에 고정되어야 하며, 몸통은 선호하는 방향을 유지하도록 유도할 수 있다. 운동학적 자코비안(Kinematic Jacobian)은 관절 변수의 변화와 작업공간 움직임 사이의 관계를 나타내며, 이를 통해 최적화 알고리즘은 작업 오차를 줄이기 위한 전신의 협조된 조정을 계산할 수 있다.

운동학적 실행 가능성(Kinematic Feasibility)은 움직임 전체에 걸쳐 관절 위치, 속도, 그리고 경우에 따라 가속도 한계를 부과한다. 최적화기는 기계적 가동 범위를 초과하거나 비현실적인 움직임 속도를 요구하는 구성을 방지해야 한다. 추가적인 자세 비용(Posture Cost)을 통해 관절이 특이점(Singularity)이나 극단적인 구성에서 멀어지도록 할 수 있다. 여유 자유도(Redundant Degrees of Freedom)는 동일한 작업을 여러 관절 구성으로 수행할 수 있게 하므로 부차적인 목표를 만족할 수 있는 최적화 자유도를 제공한다.

균형(Balance)은 휴머노이드 움직임을 정의하는 핵심 제약조건 중 하나이다. 느린 준정적 움직임(Quasi-Static Movement)에서는 투영된 질량중심이 활성 접촉(Active Contact)에 의해 형성된 지지 다각형(Support Polygon) 내부에 유지되도록 할 수 있다. 보다 동적인 움직임에서는 가속도와 운동량이 안정성에 영향을 주기 때문에 이러한 단순 기준만으로는 충분하지 않다. 따라서 전신 최적화는 영 모멘트 점(Zero Moment Point, ZMP), 중심 운동량(Centroidal Momentum), 접촉 렌치 제약조건(Contact Wrench Constraint)과 같은 동적 물리량을 포함하는 경우가 많다.

중심 동역학(Centroidal Dynamics)은 전신 움직임과 외부 접촉력 사이를 연결하는 축소 표현(Reduced Representation)을 제공한다. 로봇의 질량중심 가속도는 중력과 발, 손 또는 기타 접촉점을 통해 작용하는 힘에 의해 결정되며, 각운동량(Angular Momentum)의 변화는 관련 모멘트에 의해 결정된다. 이러한 물리량을 최적화하면 모든 단계에서 완전한 강체 동역학(Full Rigid-Body Dynamics)을 직접 계산하지 않고도 균형을 고려할 수 있다.

더 높은 물리적 충실도를 요구하는 정식화에서는 다물체 운동방정식(Multibody Equations of Motion)을 직접 적용한다. 이 방정식은 로봇의 질량 행렬(Mass Matrix), 코리올리 및 원심 효과(Coriolis and Centrifugal Effects), 중력, 접촉 자코비안(Contact Jacobian)을 통해 일반화 위치, 속도, 가속도, 액추에이터 토크, 접촉력 사이의 관계를 정의한다. 이러한 정식화는 동역학적으로 일관된 궤적을 생성할 수 있지만 자유도가 많은 휴머노이드에서는 대규모 비선형 최적화 문제를 형성한다.

접촉(Contact)은 연속적인 결정과 이산적인 결정(Discrete Decision)을 모두 포함한다. 발이 지지 상태(Stance)에 있을 때에는 지지 표면에 대해 위치가 일관되게 유지되어야 하고 적절한 접촉력이 존재해야 한다. 스윙 상태(Swing)에서는 해당 접촉이 사라지고 발이 새로운 지지 영역으로 이동해야 한다. 따라서 보행, 등반, 다중 접촉 매니퓰레이션(Multi-Contact Manipulation)은 연속 동역학과 이산적인 접촉 모드 변화가 상호작용하는 하이브리드 시스템(Hybrid System)을 형성한다.

접촉력은 물리적인 실행 가능 조건(Physical Feasibility Condition)을 만족해야 한다. 단방향 지면 접촉(Unilateral Ground Contact)은 로봇을 밀 수 있지만 지면 방향으로 당길 수는 없다. 접선력(Tangential Force)은 미끄러짐을 방지하기 위해 마찰 한계(Friction Limit) 안에 있어야 하며, 압력 중심(Center of Pressure) 또는 접촉 렌치 제약조건은 발 전체에 하중이 어떻게 분포하는지를 제한할 수 있다. 이러한 제약을 쉽게 최적화하기 위해 마찰 원뿔(Friction Cone)을 볼록 피라미드(Convex Pyramid)로 근사하는 경우가 많다.

발걸음 배치(Footstep Placement)와 전신 움직임은 밀접하게 결합되어 있다. 발을 이동하면 미래의 지지 형상(Support Geometry)이 변화하며, 질량중심과 몸통 역시 발걸음 전후와 실행 중에 적절하게 이동해야 한다. 계층적 아키텍처(Hierarchical Architecture)는 먼저 대략적인 발걸음을 생성한 뒤 전체 신체 궤적을 최적화할 수 있다. 보다 통합적인 접근법은 접촉, 몸체 상태, 힘을 함께 최적화하여 더 우수한 움직임을 생성할 수 있지만 계산 비용은 크게 증가한다.

휴머노이드의 팔은 단순한 매니퓰레이션 장치(Manipulation Device)가 아니라 전신 균형과 운동량 조절에 중요한 역할을 할 수 있다. 팔 움직임은 몸통 회전을 보상하고, 각운동량을 감소시키며, 도달 가능성을 향상시키거나 환경과의 접촉을 준비할 수 있다. 전신 최적화는 이러한 여유 자유도를 자동으로 활용하여 상체 움직임을 독립적인 부가 동작으로 처리하는 대신 팔, 몸통, 골반, 다리의 움직임을 통합적으로 조정한다.

충돌 회피(Collision Avoidance)는 환경과 로봇 자체를 모두 고려해야 한다. 보행 중 다리끼리 충돌하거나 팔이 몸통과 교차할 수 있으며, 목표에 손을 뻗거나 회전할 때 신체가 주변 구조물과 충돌할 수도 있다. 충돌 형상(Collision Geometry)은 캡슐(Capsule), 구(Sphere), 볼록 프리미티브(Convex Primitive), 메시(Mesh), 부호 거리 모델(Signed-Distance Model) 등으로 표현할 수 있다. 미분 가능한 거리 정보(Differentiable Distance Information)는 최적화된 구성을 충돌 영역에서 밀어내는 그래디언트를 제공하므로 특히 유용하다.

환경 접촉(Environmental Contact)을 의도적으로 활용할 수도 있다. 휴머노이드는 벽, 난간, 사다리 또는 지지 구조물에 손을 접촉시켜 안정성을 향상시키거나 그렇지 않으면 불가능한 움직임을 수행할 수 있다. 이는 일반적인 발걸음 계획을 다중 접촉 전신 계획(Multi-Contact Whole-Body Planning)으로 확장한다. 이 경우 최적화기는 마찰과 도달 가능성 제약조건을 만족하면서 여러 접촉점에 대한 실행 가능한 신체 구성과 힘 분배(Force Distribution)를 결정해야 한다.

전신 움직임 최적화는 강한 비볼록성(Nonconvexity)을 가진다. 순기구학, 회전, 충돌 회피, 접촉 전환(Contact Switching), 마찰, 비선형 동역학은 여러 국소 해(Local Solution)와 실행 불가능 영역(Infeasible Region)을 생성한다. 따라서 초기화(Initialization)가 수렴에 큰 영향을 미친다. 초기 궤적은 역기구학(Inverse Kinematics), 발걸음 계획기(Footstep Planner), 모션 라이브러리(Motion Library), 단순화된 중심 동역학 모델, 이전 최적화 결과 또는 수동으로 구성된 기준 움직임에서 얻을 수 있다.

정식화 방식에 따라 순차 이차 계획법(Sequential Quadratic Programming), 순차 볼록 최적화(Sequential Convex Optimization), 직접 콜로케이션(Direct Collocation), 미분 동적 계획법(Differential Dynamic Programming) 및 관련 비선형 최적화 기법을 적용할 수 있다. 직접 전사 방법(Direct Transcription Method)은 시간에 따른 상태와 제어 입력을 최적화 변수로 처리하며, 슈팅 방법(Shooting Method)은 제어 입력으로부터 동역학을 적분하여 상태를 생성한다. 각각의 표현은 희소성(Sparsity), 제약조건 처리, 수치 조건화(Numerical Conditioning), 계산 효율성 측면에서 서로 다른 절충 관계를 가진다.

직접 콜로케이션(Direct Collocation)은 궤적 전체에 걸쳐 동역학과 제약조건을 적용해야 하는 경우 특히 유용하다. 상태와 제어 입력을 이산적인 절점(Knot Point)에 정의하고, 절점 사이에서도 동역학적 일관성이 유지되도록 추가 방정식을 적용한다. 이를 통해 연속시간 최적 제어(Continuous-Time Optimal Control) 문제를 구조화된 비선형 계획 문제(Structured Nonlinear Program)로 변환할 수 있다. 각 시간 단계가 주로 인접한 궤적 상태와 상호작용하기 때문에 희소 최적화기(Sparse Solver)를 효과적으로 활용할 수 있다.

단순한 가중합(Weighted Sum)으로 작업 우선순위를 적절하게 표현하기 어려운 경우 계층적 최적화(Hierarchical Optimization)를 사용할 수 있다. 접촉 유지나 낙상 방지(Fall Prevention)는 선호 자세 추종보다 높은 우선순위를 가질 수 있으며, 손의 정확도는 작은 관절 움직임을 최소화하는 것보다 중요할 수 있다. 사전식 최적화(Lexicographic Optimization) 또는 계층적 정식화는 낮은 우선순위 목표가 핵심 제약조건을 악화시키지 않도록 하지만 더욱 정교한 수치 최적화 방법이 필요할 수 있다.

궤적 지속시간(Trajectory Duration)과 접촉 타이밍 역시 최적화 변수가 될 수 있다. 느린 움직임은 필요한 힘을 줄이고 안정성을 향상시킬 수 있는 반면 빠른 움직임은 더 큰 운동량과 액추에이터 노력을 요구할 수 있다. 공간적 움직임과 타이밍을 함께 최적화하면 고정된 일정에서는 불가능했던 움직임을 발견할 수 있다. 그러나 가변 타이밍(Variable Timing)은 동역학, 접촉, 궤적 이산화 사이에 추가적인 비선형 결합을 발생시킨다.

최적화된 궤적은 로봇 상태, 지형 형상, 마찰, 동역학 모델의 추정값에 의존하므로 불확실성(Uncertainty)은 실제 실행을 어렵게 만든다. 강건한 정식화(Robust Formulation)는 안전 여유(Safety Margin), 불확실성 집합(Uncertainty Set), 위험 민감 비용(Risk-Sensitive Cost)을 도입할 수 있다. 이동 시간 구간 최적화(Receding-Horizon Optimization)는 또 다른 대응 방법으로, 계획된 움직임의 초기 일부만 실행한 뒤 상태를 다시 측정하고 갱신된 정보를 이용하여 나머지 궤적을 다시 계산한다.

따라서 모델 예측 제어(Model Predictive Control, MPC)는 전신 최적화를 오프라인 계획(Offline Planning)에서 피드백 제어(Feedback Control) 방향으로 확장할 수 있다. 각 제어 주기에서 현재 로봇 상태와 접촉 조건을 이용하여 유한 시간 구간의 전신 최적화 문제를 해결한다. 첫 번째 제어 동작을 실행한 뒤 계획 구간을 앞으로 이동시킨다. 실제 휴머노이드 시스템에 적합한 제어 주파수를 달성하려면 웜 스타트(Warm Start), 축소 차수 모델(Reduced-Order Model), 희소 계산(Sparse Computation), 병렬 하드웨어(Parallel Hardware)가 중요하다.

일반적인 실제 아키텍처는 복잡성을 여러 계층으로 분산한다. 전역 계획기(Global Planner)는 휴머노이드가 이동해야 할 위치를 결정하고, 발걸음 또는 접촉 계획기(Contact Planner)는 적절한 지지 위치를 결정하며, 중심 동역학 계획기(Centroidal Planner)는 대략적인 몸체와 운동량 변화를 결정한다. 이후 전신 최적화가 일관된 관절 수준 움직임(Joint-Level Motion)을 생성한다. 하위 수준 제어기(Low-Level Controller)는 외란과 모델링 오차에 빠르게 대응하면서 생성된 기준값을 추종한다.

수치 최적화가 성공적으로 수렴하더라도 검증(Validation)은 필수적이다. 생성된 움직임에 대해 충돌, 관절 한계, 액추에이터 한계, 마찰 실행 가능성(Friction Feasibility), 접촉 안정성(Contact Stability), 동역학적 일관성(Dynamic Consistency), 충분한 환경 여유(Environmental Clearance)를 확인해야 한다. 실제 실행 전에 시뮬레이션을 통해 상태 추정 오차나 접촉 불확실성에 대한 민감도를 확인할 수도 있다. 수치적 수렴(Numerical Convergence)만으로 움직임의 물리적 안전성과 강건성이 보장되는 것은 아니다.

전신 움직임 최적화는 궁극적으로 휴머노이드를 서로 독립적으로 제어되는 여러 팔다리의 집합이 아니라 하나의 결합된 물리 시스템(Coupled Physical System)으로 다룬다. 운동학, 동역학, 균형, 접촉, 충돌 회피, 작업 목표, 힘, 타이밍을 통합적으로 고려함으로써 복잡한 환경에서 보행(Walking), 도달(Reaching), 매니퓰레이션(Manipulation), 등반(Climbing), 다중 접촉 행동(Multi-Contact Behavior)을 생성하기 위한 체계적인 기반을 제공한다.

##  

## 04.09. GPU Accelerated Trajectory Optimization [w/Code]

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

GPU-accelerated trajectory optimization uses massively parallel computation to reduce the time required for robot motion generation. Conventional trajectory optimization repeatedly evaluates dynamics, collision conditions, costs, gradients, and candidate trajectories, creating substantial computational demand for high-dimensional robots. GPUs provide thousands of parallel execution units that can process many independent trajectory operations simultaneously.

The opportunity for GPU acceleration arises from the structure of trajectory optimization itself. A trajectory contains many time steps, robot states, control inputs, collision queries, and cost terms that can often be evaluated independently or with limited coupling. Instead of processing these calculations sequentially on a CPU, GPU implementations distribute them across parallel threads, allowing large batches of trajectory-related computations to execute concurrently.

A robot trajectory may be represented as states \\(x_0,x_1,\\ldots,x_T\\) and controls \\(u_0,u_1,\\ldots,u_{T-1}\\). Optimization repeatedly modifies these variables to minimize an objective \\(J(X,U)\\) while satisfying dynamics, collision, actuator, and task constraints. As the horizon length, robot degrees of freedom, and environmental complexity increase, the number of calculations grows rapidly, making parallel hardware increasingly valuable.

Collision checking is one of the most important acceleration targets. During optimization, hundreds or thousands of robot configurations may need to be evaluated against complex environments. GPU kernels can perform distance calculations for many robot links, obstacle primitives, trajectory states, or sampled configurations simultaneously. Parallel collision evaluation can therefore remove a major computational bottleneck in manipulation, mobile robotics, and whole-body planning.

Signed-distance fields and voxel representations are particularly suitable for GPU processing. Environmental geometry can be stored in regular three-dimensional grids, allowing many trajectory points to query obstacle distances concurrently. Distance gradients can also be evaluated in parallel and supplied to gradient-based optimization. GPU texture and memory mechanisms can further support efficient spatial access when data organization is designed for coherent queries.

Forward kinematics is another highly parallel workload. For each trajectory state, robot link transforms must often be computed before collision distances or task-space errors can be evaluated. Although kinematic chains contain internal dependencies, calculations across trajectory steps, candidate trajectories, or robots are largely independent. Batched forward kinematics therefore enables substantial acceleration, especially for manipulators and humanoids with many joints.

Dynamics computation can similarly benefit from batching. Model-based trajectory optimization may repeatedly evaluate state transitions, rigid-body dynamics, Jacobians, mass matrices, or contact quantities across the planning horizon. Parallel computation across time steps or candidate trajectories reduces the cost of these repeated model evaluations. The benefit becomes particularly significant when many rollouts must be simulated during stochastic optimization.

Sampling-based optimal control methods such as MPPI are naturally aligned with GPU architectures. Thousands of perturbed control sequences can be generated and rolled forward independently through the robot model. Each rollout computes its own trajectory cost, after which parallel reduction operations determine importance weights and control updates. This high degree of independence allows GPU acceleration to transform computationally expensive sampling into practical real-time planning.

Gradient-based trajectory optimization can also exploit GPU parallelism. Cost terms associated with different trajectory states, links, obstacles, or tasks can be evaluated concurrently. Automatic differentiation frameworks can compute gradients through batched robot models and cost functions, reducing the need for manually derived derivatives. The resulting gradients can then be combined by the optimization algorithm to update the complete trajectory.

Modern differentiable robotics frameworks increasingly represent kinematics, dynamics, collision models, and costs as tensor operations. A batch dimension may correspond to different trajectories, time steps, robot configurations, or planning problems. GPU tensor libraries execute these operations efficiently using highly optimized kernels. This representation also integrates naturally with automatic differentiation, enabling rapid development of differentiable trajectory optimizers.

Batch optimization is particularly useful when multiple candidate solutions are desirable. Because trajectory optimization is nonconvex, a single initialization can converge to a poor local minimum. Several initial trajectories can instead be optimized simultaneously on the GPU. Different candidates may pass around obstacles in different directions or represent different motion strategies, after which the lowest-cost feasible solution can be selected.

Parallel multi-start optimization therefore improves both computation and planning robustness. A CPU implementation might solve candidate trajectories sequentially, making extensive multi-start search expensive. A GPU can process many candidates in one batch, although memory requirements increase. This approach does not eliminate local minima, but it increases the probability that at least one initialization reaches a useful solution within the available planning time.

GPU acceleration is also valuable for trajectory optimization involving learned models. Neural network dynamics, learned collision predictors, neural signed-distance fields, and learned cost functions are already designed for tensor computation. These components can remain on the GPU while trajectory variables are optimized, avoiding repeated transfer between CPU and accelerator. This enables hybrid planners combining analytical robot models with learned representations.

Memory transfer is a critical design issue. Moving states, geometry, gradients, and costs repeatedly between CPU and GPU can eliminate much of the expected acceleration. High-performance implementations therefore keep frequently used robot models, environmental representations, trajectory tensors, and optimization buffers resident in GPU memory. Only essential planning commands and final results should cross the host-device boundary whenever possible.

Memory layout also strongly influences performance. GPUs achieve high throughput when neighboring threads access contiguous or predictable memory locations. Trajectory data should therefore be organized to support coalesced access across time steps, degrees of freedom, or batch elements. Poor layout can produce excessive memory transactions and reduce utilization even when the underlying mathematical computation is highly parallel.

Kernel launch overhead becomes relevant for optimization algorithms composed of many small operations. Launching separate GPU kernels for every minor cost or transformation can create synchronization and scheduling overhead. Kernel fusion, batched operations, persistent buffers, and computational graphs can reduce this overhead. Effective GPU trajectory optimization therefore requires algorithm-hardware co-design rather than simply transferring CPU code to an accelerator.

Not every part of trajectory optimization parallelizes equally well. Sequential algorithms may contain recursions, line searches, sparse factorizations, or dependency chains that limit GPU utilization. iLQR, for example, includes a backward recursion whose time steps are inherently coupled, although rollout evaluation and derivative computation can still be accelerated. Algorithms should therefore be decomposed into parallel and sequential components rather than assuming uniform acceleration.

Sparse nonlinear programming presents similar challenges. Large trajectory problems often produce sparse Jacobian and Hessian matrices whose structure is advantageous on CPUs with mature sparse solvers. GPUs can accelerate selected linear algebra operations, but irregular sparsity and repeated synchronization may reduce efficiency. Hybrid CPU-GPU architectures can therefore be preferable, with the GPU evaluating expensive models and derivatives while the CPU coordinates optimization or sparse factorization.

Precision is another practical consideration. GPUs provide very high throughput for lower-precision arithmetic, but trajectory optimization can be sensitive to numerical conditioning. Single precision may be adequate for many geometric calculations and sampling-based planners, whereas difficult nonlinear optimization may require double precision in critical operations. Mixed-precision strategies can combine fast approximate evaluation with higher-precision computation where convergence or constraint accuracy demands it.

Real-time robotics requires predictable latency rather than high average throughput alone. A planner operating at tens or hundreds of hertz must complete optimization before its control deadline. GPU execution time can vary with batch size, memory allocation, synchronization, and competing workloads. Preallocated buffers, fixed-size batches, asynchronous pipelines, and bounded iteration counts can improve deterministic behavior and reduce worst-case planning latency.

For mobile robots, GPU-accelerated optimization can evaluate large numbers of local trajectories while simultaneously considering costmaps, dynamics, obstacle clearance, and path tracking. For manipulators, batched kinematics and collision checking can accelerate high-dimensional planning. Humanoid and legged robots benefit from parallel evaluation of contacts, body geometry, dynamics, and candidate motions, where conventional optimization can otherwise become computationally prohibitive.

Receding-horizon planning benefits directly from acceleration because a new optimization problem must be solved repeatedly as the robot moves. The previous solution can be shifted forward and used as a warm start, while the GPU rapidly reevaluates the updated state and environment. This combination of warm starting and parallel computation enables increasingly sophisticated trajectory optimization to operate inside model predictive planning and control loops.

GPU acceleration changes the practical design space of robot planning rather than changing the mathematical definition of trajectory optimization itself. More samples, longer horizons, richer collision geometry, multiple initializations, and more detailed robot models become computationally affordable. The planner can therefore evaluate a broader set of alternatives or use higher-fidelity models within the same latency budget available to a conventional implementation.

Within optimization-based motion planning, GPU acceleration provides the computational infrastructure for scaling algorithms such as MPPI, differentiable trajectory optimization, parallel collision-aware planning, and batched multi-start optimization. Its central principle is to expose parallelism across trajectories, time steps, robot links, costs, and environment queries while minimizing synchronization and data movement. Properly designed, this architecture enables high-dimensional trajectory optimization to approach real-time robotic operation.

GPU 가속 궤적 최적화(GPU-Accelerated Trajectory Optimization)는 대규모 병렬 연산(Massively Parallel Computation)을 활용하여 로봇 움직임 생성(Robot Motion Generation)에 필요한 시간을 단축한다. 기존 궤적 최적화는 동역학, 충돌 조건, 비용, 그래디언트(Gradient), 후보 궤적을 반복적으로 평가하므로 고차원 로봇(High-Dimensional Robot)에서는 상당한 계산량이 발생한다. GPU는 수천 개의 병렬 실행 유닛(Parallel Execution Unit)을 제공하여 서로 독립적인 다수의 궤적 관련 연산을 동시에 처리할 수 있다.

GPU 가속의 가능성은 궤적 최적화 자체의 구조에서 비롯된다. 하나의 궤적에는 많은 시간 단계(Time Step), 로봇 상태(Robot State), 제어 입력(Control Input), 충돌 질의(Collision Query), 비용 항(Cost Term)이 포함되며, 이들 중 상당수는 서로 독립적이거나 제한적으로만 결합되어 평가될 수 있다. CPU에서 이러한 계산을 순차적으로 처리하는 대신 GPU 구현은 이를 병렬 스레드(Parallel Thread)에 분배하여 대규모 궤적 관련 계산을 동시에 실행한다.

로봇 궤적은 상태 \\(x_0,x_1,\\ldots,x_T\\)와 제어 입력 \\(u_0,u_1,\\ldots,u_{T-1}\\)로 표현할 수 있다. 최적화는 동역학, 충돌, 액추에이터(Actuator), 작업 제약조건(Task Constraint)을 만족하면서 목적함수 \\(J(X,U)\\)를 최소화하도록 이러한 변수들을 반복적으로 수정한다. 계획 구간(Horizon Length), 로봇 자유도(Degrees of Freedom), 환경 복잡도가 증가할수록 계산량이 빠르게 증가하므로 병렬 하드웨어(Parallel Hardware)의 가치도 더욱 커진다.

충돌 검사(Collision Checking)는 GPU 가속의 가장 중요한 대상 중 하나이다. 최적화 과정에서는 수백 또는 수천 개의 로봇 구성을 복잡한 환경과 비교하여 평가해야 할 수 있다. GPU 커널(GPU Kernel)은 여러 로봇 링크, 장애물 프리미티브(Obstacle Primitive), 궤적 상태 또는 샘플링된 구성을 대상으로 거리 계산을 동시에 수행할 수 있다. 따라서 병렬 충돌 평가는 매니퓰레이션(Manipulation), 이동 로봇(Mobile Robotics), 전신 계획(Whole-Body Planning)에서 주요 계산 병목을 제거할 수 있다.

부호 거리장(Signed-Distance Field)과 복셀 표현(Voxel Representation)은 GPU 처리에 특히 적합하다. 환경 형상을 규칙적인 3차원 격자에 저장하면 많은 궤적 지점이 장애물 거리를 동시에 질의할 수 있다. 거리 그래디언트(Distance Gradient) 역시 병렬로 계산하여 그래디언트 기반 최적화(Gradient-Based Optimization)에 제공할 수 있다. 데이터 구성을 일관된 공간 접근에 적합하도록 설계하면 GPU의 텍스처(Texture) 및 메모리 메커니즘을 이용해 효율적인 공간 질의를 지원할 수 있다.

순기구학(Forward Kinematics) 역시 높은 병렬성을 갖는 작업이다. 각 궤적 상태에서는 충돌 거리나 작업공간 오차(Task-Space Error)를 평가하기 전에 로봇 링크 변환(Link Transform)을 계산해야 하는 경우가 많다. 운동학적 체인(Kinematic Chain) 내부에는 의존성이 존재하지만 서로 다른 궤적 단계, 후보 궤적 또는 로봇 사이의 계산은 대부분 독립적이다. 따라서 배치 순기구학(Batched Forward Kinematics)은 특히 관절이 많은 매니퓰레이터와 휴머노이드에서 상당한 가속 효과를 제공한다.

동역학 계산(Dynamics Computation)도 배치 처리(Batching)를 통해 가속할 수 있다. 모델 기반 궤적 최적화(Model-Based Trajectory Optimization)는 계획 구간 전체에서 상태 전이(State Transition), 강체 동역학(Rigid-Body Dynamics), 자코비안(Jacobian), 질량 행렬(Mass Matrix), 접촉 관련 물리량을 반복적으로 계산할 수 있다. 시간 단계 또는 후보 궤적에 걸쳐 이러한 계산을 병렬화하면 반복적인 모델 평가 비용을 줄일 수 있으며, 확률적 최적화에서 많은 롤아웃(Rollout)을 시뮬레이션해야 할 때 특히 효과적이다.

MPPI와 같은 샘플링 기반 최적 제어(Sampling-Based Optimal Control) 방법은 GPU 아키텍처와 자연스럽게 잘 결합된다. 수천 개의 섭동된 제어 시퀀스(Perturbed Control Sequence)를 생성하여 로봇 모델을 통해 서로 독립적으로 전방 롤아웃할 수 있다. 각 롤아웃은 자체적인 궤적 비용을 계산하며, 이후 병렬 축약 연산(Parallel Reduction)을 통해 중요도 가중치(Importance Weight)와 제어 갱신을 결정한다. 이러한 높은 독립성은 계산 비용이 큰 샘플링을 실시간 계획에 활용할 수 있도록 한다.

그래디언트 기반 궤적 최적화도 GPU 병렬성을 활용할 수 있다. 서로 다른 궤적 상태, 로봇 링크, 장애물 또는 작업과 관련된 비용 항을 동시에 평가할 수 있다. 자동 미분 프레임워크(Automatic Differentiation Framework)는 배치화된 로봇 모델과 비용함수를 통해 그래디언트를 계산하여 수작업으로 미분식을 유도해야 하는 부담을 줄인다. 이후 최적화 알고리즘은 생성된 그래디언트를 결합하여 전체 궤적을 갱신할 수 있다.

현대의 미분 가능 로보틱스 프레임워크(Differentiable Robotics Framework)는 운동학, 동역학, 충돌 모델, 비용함수를 점차 텐서 연산(Tensor Operation)으로 표현하고 있다. 배치 차원(Batch Dimension)은 서로 다른 궤적, 시간 단계, 로봇 구성 또는 계획 문제를 나타낼 수 있다. GPU 텐서 라이브러리(Tensor Library)는 고도로 최적화된 커널을 사용하여 이러한 연산을 효율적으로 수행한다. 또한 이러한 표현은 자동 미분과 자연스럽게 통합되어 미분 가능 궤적 최적화기(Differentiable Trajectory Optimizer)를 빠르게 개발할 수 있도록 한다.

여러 후보 해(Candidate Solution)가 필요한 경우 배치 최적화(Batch Optimization)는 특히 유용하다. 궤적 최적화는 비볼록(Nonconvex)이므로 하나의 초기값만 사용하면 좋지 않은 국소 최솟값(Local Minimum)에 수렴할 수 있다. 대신 여러 초기 궤적을 GPU에서 동시에 최적화할 수 있다. 각 후보는 장애물을 서로 다른 방향으로 우회하거나 서로 다른 움직임 전략을 나타낼 수 있으며, 최종적으로 실행 가능한 해 가운데 비용이 가장 낮은 궤적을 선택할 수 있다.

따라서 병렬 다중 시작 최적화(Parallel Multi-Start Optimization)는 계산 성능뿐만 아니라 계획 강건성(Planning Robustness)도 향상시킨다. CPU 구현에서는 후보 궤적을 순차적으로 최적화해야 할 수 있어 광범위한 다중 시작 탐색이 많은 시간을 요구한다. GPU에서는 메모리 사용량이 증가하는 대신 여러 후보를 하나의 배치로 처리할 수 있다. 이러한 방법이 국소 최솟값 문제 자체를 제거하지는 않지만 제한된 계획 시간 내에 적어도 하나의 초기값이 유용한 해에 도달할 가능성을 높인다.

GPU 가속은 학습 기반 모델(Learned Model)을 포함하는 궤적 최적화에서도 유용하다. 신경망 동역학(Neural Network Dynamics), 학습된 충돌 예측기(Learned Collision Predictor), 신경 부호 거리장(Neural Signed-Distance Field), 학습된 비용함수(Learned Cost Function)는 본래 텐서 계산에 적합하게 설계되어 있다. 이러한 구성 요소와 궤적 변수를 GPU에 유지하면 CPU와 가속기 사이의 반복적인 데이터 전송을 피할 수 있다. 이를 통해 해석적 로봇 모델과 학습 표현을 결합한 하이브리드 계획기(Hybrid Planner)를 구성할 수 있다.

메모리 전송(Memory Transfer)은 매우 중요한 설계 요소이다. 상태, 형상, 그래디언트, 비용 데이터를 CPU와 GPU 사이에서 반복적으로 이동시키면 기대했던 가속 효과의 상당 부분이 사라질 수 있다. 따라서 고성능 구현에서는 자주 사용되는 로봇 모델, 환경 표현, 궤적 텐서(Trajectory Tensor), 최적화 버퍼(Optimization Buffer)를 가능한 한 GPU 메모리에 상주시킨다. 호스트-디바이스 경계(Host-Device Boundary)를 통과하는 데이터는 필수적인 계획 명령과 최종 결과 정도로 최소화하는 것이 바람직하다.

메모리 배치(Memory Layout) 역시 성능에 큰 영향을 미친다. GPU는 인접한 스레드가 연속적이거나 예측 가능한 메모리 위치에 접근할 때 높은 처리량을 얻는다. 따라서 궤적 데이터는 시간 단계, 자유도 또는 배치 요소에 걸쳐 병합 메모리 접근(Coalesced Memory Access)을 지원하도록 구성해야 한다. 수학적 계산 자체가 높은 병렬성을 갖더라도 잘못된 메모리 배치는 과도한 메모리 트랜잭션을 발생시키고 GPU 활용률(Utilization)을 크게 떨어뜨릴 수 있다.

많은 작은 연산으로 구성된 최적화 알고리즘에서는 커널 실행 오버헤드(Kernel Launch Overhead)도 중요해진다. 작은 비용 항이나 변환 연산마다 별도의 GPU 커널을 실행하면 동기화(Synchronization)와 스케줄링 오버헤드가 증가할 수 있다. 커널 융합(Kernel Fusion), 배치 연산, 지속형 버퍼(Persistent Buffer), 계산 그래프(Computational Graph)를 이용하면 이러한 오버헤드를 줄일 수 있다. 따라서 효과적인 GPU 궤적 최적화는 단순히 CPU 코드를 가속기로 이전하는 것이 아니라 알고리즘-하드웨어 공동 설계(Algorithm-Hardware Co-Design)를 필요로 한다.

궤적 최적화의 모든 부분이 동일하게 병렬화되는 것은 아니다. 순차 알고리즘(Sequential Algorithm)에는 재귀 계산(Recursion), 선 탐색(Line Search), 희소 행렬 분해(Sparse Factorization), 의존성 체인(Dependency Chain)이 포함될 수 있어 GPU 활용을 제한한다. 예를 들어 iLQR에는 시간 단계가 본질적으로 서로 결합된 역방향 재귀(Backward Recursion)가 포함되지만 롤아웃 평가와 미분 계산은 여전히 가속할 수 있다. 따라서 모든 계산이 동일하게 가속될 것이라고 가정하기보다 알고리즘을 병렬 요소와 순차 요소로 분해해야 한다.

희소 비선형 계획법(Sparse Nonlinear Programming)도 유사한 문제를 가진다. 대규모 궤적 문제에서는 희소 자코비안(Sparse Jacobian)과 헤시안 행렬(Hessian Matrix)이 생성되는 경우가 많으며, 이러한 구조는 성숙한 희소 최적화기(Sparse Solver)를 사용하는 CPU에서 유리할 수 있다. GPU가 일부 선형대수 연산을 가속할 수 있지만 불규칙한 희소성과 반복적인 동기화는 효율성을 떨어뜨릴 수 있다. 따라서 GPU가 계산량이 큰 모델과 미분을 평가하고 CPU가 최적화 또는 희소 행렬 분해를 담당하는 하이브리드 CPU-GPU 아키텍처(Hybrid CPU-GPU Architecture)가 더 적합할 수 있다.

연산 정밀도(Precision) 역시 실용적으로 중요한 요소이다. GPU는 낮은 정밀도의 산술 연산에서 매우 높은 처리량을 제공하지만 궤적 최적화는 수치적 조건(Numerical Conditioning)에 민감할 수 있다. 단정밀도(Single Precision)는 많은 기하학적 계산과 샘플링 기반 계획기에 충분할 수 있지만 어려운 비선형 최적화에서는 핵심 연산에 배정밀도(Double Precision)가 필요할 수 있다. 혼합 정밀도 전략(Mixed-Precision Strategy)은 빠른 근사 평가와 수렴 또는 제약 정확도가 필요한 부분의 높은 정밀도 계산을 결합할 수 있다.

실시간 로보틱스(Real-Time Robotics)에서는 높은 평균 처리량만큼 예측 가능한 지연시간(Predictable Latency)이 중요하다. 수십 또는 수백 Hz로 동작하는 계획기는 제어 데드라인(Control Deadline) 이전에 최적화를 완료해야 한다. GPU 실행 시간은 배치 크기, 메모리 할당, 동기화, 다른 작업과의 자원 경쟁에 따라 달라질 수 있다. 사전 할당 버퍼(Preallocated Buffer), 고정 크기 배치(Fixed-Size Batch), 비동기 파이프라인(Asynchronous Pipeline), 제한된 반복 횟수(Bounded Iteration Count)를 사용하면 결정론적 동작을 개선하고 최악 조건 계획 지연시간(Worst-Case Planning Latency)을 줄일 수 있다.

이동 로봇(Mobile Robot)의 경우 GPU 가속 최적화는 비용맵(Costmap), 동역학, 장애물 여유 거리, 경로 추종(Path Tracking)을 동시에 고려하면서 많은 수의 국소 궤적을 평가할 수 있다. 매니퓰레이터에서는 배치 운동학(Batched Kinematics)과 충돌 검사를 통해 고차원 계획을 가속할 수 있다. 휴머노이드와 다족 로봇(Legged Robot)은 접촉, 신체 형상, 동역학, 후보 움직임을 병렬로 평가함으로써 기존 최적화 방식에서는 계산량이 지나치게 커질 수 있는 문제를 효율적으로 처리할 수 있다.

이동 시간 구간 계획(Receding-Horizon Planning)은 로봇이 움직이는 동안 새로운 최적화 문제를 반복적으로 해결해야 하므로 GPU 가속의 직접적인 이점을 얻는다. 이전 해를 앞으로 이동시켜 웜 스타트(Warm Start)로 사용하고, GPU를 통해 갱신된 로봇 상태와 환경을 빠르게 재평가할 수 있다. 웜 스타트와 병렬 계산의 결합은 더욱 정교한 궤적 최적화를 모델 예측 계획(Model Predictive Planning) 및 제어 루프(Control Loop) 내부에서 실행할 수 있도록 한다.

GPU 가속은 궤적 최적화의 수학적 정의 자체를 변경하는 것이 아니라 로봇 계획의 실질적인 설계 공간(Practical Design Space)을 확장한다. 동일한 지연시간 예산(Latency Budget) 안에서 더 많은 샘플, 더 긴 계획 구간, 더 정밀한 충돌 형상, 다수의 초기값, 더 상세한 로봇 모델을 사용할 수 있게 된다. 따라서 계획기는 기존 구현과 동일한 계산 시간 안에서 더 많은 대안을 평가하거나 더 높은 충실도의 모델(High-Fidelity Model)을 활용할 수 있다.

최적화 기반 움직임 계획(Optimization-Based Motion Planning)에서 GPU 가속은 MPPI, 미분 가능 궤적 최적화(Differentiable Trajectory Optimization), 병렬 충돌 인식 계획(Parallel Collision-Aware Planning), 배치 다중 시작 최적화(Batched Multi-Start Optimization)와 같은 알고리즘을 확장하기 위한 계산 인프라(Computational Infrastructure)를 제공한다. 핵심 원리는 궤적, 시간 단계, 로봇 링크, 비용, 환경 질의 전반에서 병렬성을 최대한 노출하면서 동기화와 데이터 이동을 최소화하는 것이다. 적절하게 설계된 이러한 아키텍처는 고차원 궤적 최적화를 실시간 로봇 운용(Real-Time Robotic Operation)에 가까운 수준으로 구현할 수 있게 한다.

##  

## 04.10. Optimization Planning for Cargo UAV Trajectory [w/Code]

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

Cargo UAV trajectory planning is an optimization problem in which the aircraft must transport a payload between locations while satisfying flight dynamics, vehicle limits, environmental constraints, and mission objectives. Unlike geometric path planning that searches only for collision-free positions, optimization-based planning determines a time-dependent trajectory whose position, velocity, attitude, thrust, and sometimes energy state remain physically executable throughout the mission.

The trajectory can be represented by a sequence of states \\(x_0,x_1,\\ldots,x_T\\) and control inputs \\(u_0,u_1,\\ldots,u_{T-1}\\). A state may contain three-dimensional position, velocity, attitude, angular velocity, battery state, or payload-related variables. Controls may represent total thrust, body torques, rotor commands, or higher-level acceleration references depending on the fidelity of the UAV model used by the planner.

Cargo transport changes the optimization problem because payload mass directly affects acceleration capability, thrust demand, energy consumption, and maneuverability. A UAV carrying a heavy load generally requires larger thrust to hover and has less remaining control authority for acceleration or disturbance rejection. The planner should therefore use the actual or estimated payload mass rather than assuming the dynamic behavior of an unloaded vehicle.

The payload can also modify the vehicle\'s center of mass and inertia tensor. If the cargo is mounted away from the nominal center, rotational dynamics and actuator requirements change. Optimization based on an inaccurate mass distribution may generate trajectories that appear feasible numerically but are difficult to track physically. Payload-aware models therefore improve the consistency between planned motion and the actual aircraft response.

Suspended cargo introduces additional dynamics because the payload can swing relative to the UAV. A simplified model may represent the load as a pendulum connected by a cable, adding payload angle and angular velocity to the system state. Aggressive acceleration, braking, or turning can excite oscillation. Trajectory optimization can penalize swing angle and swing velocity while limiting maneuvers that produce unacceptable payload motion.

The objective function combines mission performance with flight quality. Typical terms penalize travel time, energy consumption, deviation from a desired route, control effort, excessive acceleration, obstacle proximity, and terminal position error. Cargo-specific costs may additionally penalize payload swing, abrupt attitude changes, high jerk, or excessive load factors. Weight selection determines the tradeoff between speed, efficiency, safety, and smoothness.

Vehicle dynamics form the central feasibility constraint. A multirotor trajectory must satisfy translational and rotational equations of motion while remaining within available thrust and torque limits. Simplified planning may use point-mass or differential-flatness models, whereas higher-fidelity optimization can incorporate full rigid-body dynamics. The appropriate model depends on mission complexity, computation budget, and required tracking accuracy.

Differential flatness is particularly useful for multirotor trajectory generation because position and yaw can often serve as flat outputs from which attitude, velocity, acceleration, and control quantities are reconstructed. This allows trajectory optimization to operate on a reduced set of variables while retaining important dynamic relationships. Polynomial or spline trajectories can then be optimized for smoothness, timing, obstacle clearance, and actuator feasibility.

Minimum-snap trajectory optimization is widely associated with aggressive multirotor motion. Piecewise polynomial segments connect waypoints while minimizing high-order derivatives such as snap, producing smooth position trajectories and corresponding control commands. For cargo UAVs, however, pure minimum-snap objectives may need additional constraints because smooth mathematical trajectories do not automatically account for payload mass, energy limits, wind, suspended-load oscillation, or operational safety margins.

Obstacle avoidance requires reasoning in three-dimensional space. Buildings, terrain, cranes, power infrastructure, trees, and other structures can restrict feasible flight corridors. Obstacles may be represented using meshes, occupancy maps, voxel grids, signed-distance fields, or convex regions. Optimization can impose clearance constraints or obstacle penalties so that the vehicle and, when necessary, its suspended payload remain separated from environmental geometry.

Convex safe corridors provide a practical interface between global path planning and continuous trajectory optimization. A coarse planner can first identify a collision-free route and construct a sequence of overlapping convex regions around it. Polynomial control points or discrete UAV states are then constrained to remain inside these regions. This converts part of the nonconvex collision-avoidance problem into simpler linear or convex constraints.

Trajectory timing is tightly coupled with feasibility. Traversing the same geometric route faster requires greater acceleration, thrust, and attitude changes, while slower flight generally reduces dynamic demand but increases mission duration and potentially energy consumed against wind. Optimization may therefore treat segment duration or total mission time as decision variables, jointly determining both the spatial route and the speed profile along it.

Energy-aware planning is especially important for cargo UAVs because payload reduces flight endurance. Battery consumption depends on vehicle mass, aerodynamic conditions, flight velocity, maneuvering, wind, and propulsion efficiency. An energy model can be included in the objective or as a constraint requiring sufficient reserve at landing. The fastest trajectory is therefore not necessarily the trajectory with the lowest energy consumption or greatest operational margin.

Wind introduces both deterministic and uncertain disturbances. A known wind field can be incorporated into the dynamic model so that the optimizer selects headings, speeds, and altitude profiles that account for predicted airflow. Strong headwinds may make a short geometric route inefficient, while favorable winds may justify a longer spatial path. Wind gradients near buildings or terrain can further influence local trajectory feasibility.

Robust trajectory planning addresses uncertainty in wind, payload properties, state estimation, and vehicle models. Conservative margins can reduce allowable thrust, enlarge obstacle clearance, or preserve additional battery reserve. More advanced approaches can optimize against disturbance sets, probabilistic constraints, or multiple environmental scenarios. The resulting trajectory may sacrifice nominal performance in exchange for improved feasibility under real operating conditions.

Operational constraints extend beyond physical collision avoidance. A cargo mission may include permitted flight corridors, altitude bands, takeoff and landing zones, approach directions, speed restrictions, or regions that should not be traversed. These constraints can be encoded as geometric regions, state bounds, temporal conditions, or mission-level constraints, allowing the optimizer to generate trajectories consistent with both vehicle capability and operational requirements.

Takeoff and landing deserve special treatment because they occur near surfaces and usually provide smaller safety margins than cruise flight. Terminal constraints may specify position, velocity, acceleration, yaw, or approach direction at the destination. For precision cargo delivery, the planner may also require a stable hover or low-velocity approach before release, ensuring that residual vehicle motion and payload swing remain within acceptable limits.

Sequential convex optimization can solve cargo UAV planning problems by repeatedly approximating nonlinear dynamics and collision constraints around a nominal trajectory. Each iteration solves a more tractable subproblem and then updates the reference trajectory. Trust regions limit changes between iterations so that local approximations remain meaningful. A good initialization from a geometric planner or previous trajectory can substantially improve convergence.

Direct transcription and direct collocation provide alternatives for higher-fidelity planning. States and controls across the flight horizon become optimization variables, while equality constraints enforce dynamic consistency between neighboring time points. Inequality constraints represent thrust, velocity, attitude, collision, and operational limits. The resulting nonlinear program can exploit temporal sparsity, although computational demand increases as model fidelity and horizon resolution grow.

Receding-horizon optimization allows the cargo UAV to adapt after flight begins. Rather than executing an entire precomputed trajectory without modification, the planner repeatedly optimizes a finite future horizon using the latest vehicle state, payload behavior, obstacle information, and wind estimate. Only an initial portion of the solution is executed before replanning, connecting trajectory optimization naturally with Model Predictive Control.

Real-time replanning becomes particularly important when moving obstacles, unexpected winds, or inaccurate payload models invalidate the original trajectory. Warm starting from the previous solution reduces optimization time, while GPU acceleration can parallelize collision queries, trajectory evaluation, sampled alternatives, or model computations. Planning latency must remain bounded because an excellent trajectory computed too late has limited value for a rapidly moving aerial vehicle.

A hierarchical architecture can distribute the overall problem across several planning levels. Mission planning selects pickup and delivery objectives, global routing identifies a broad three-dimensional corridor, trajectory optimization determines dynamically feasible motion, and the flight controller tracks the resulting references. Payload estimation, perception, localization, and health monitoring continuously provide information that can trigger trajectory adjustment or mission replanning.

Trajectory validation is required before commands are executed. The planned motion should be checked against thrust and torque limits, velocity and attitude bounds, collision clearance, battery reserve, payload stability, and terminal conditions. Higher-fidelity simulation can reveal tracking errors or load oscillations hidden by simplified planning models. Safety monitoring should remain active during execution because numerical convergence alone does not guarantee operational feasibility.

Optimization-based cargo UAV planning therefore combines route geometry, aircraft dynamics, payload behavior, energy, environmental constraints, uncertainty, and mission timing within a unified trajectory-generation framework. By optimizing where the UAV flies, how fast it moves, and how its controls evolve through time, the planner can produce cargo missions that are not merely collision-free but dynamically feasible, energy-aware, smooth, and suitable for repeated real-world replanning.

화물 무인항공기(Cargo UAV) 궤적 계획은 항공기가 비행 동역학(Flight Dynamics), 기체 한계(Vehicle Limits), 환경 제약조건(Environmental Constraints), 임무 목표(Mission Objectives)를 만족하면서 출발지에서 목적지까지 화물을 운송해야 하는 최적화 문제(Optimization Problem)이다. 단순히 충돌이 없는 위치만 탐색하는 기하학적 경로 계획(Geometric Path Planning)과 달리, 최적화 기반 계획은 위치, 속도, 자세, 추력(Thrust), 그리고 경우에 따라 에너지 상태(Energy State)가 임무 전체에서 물리적으로 실행 가능한 시간 의존적 궤적(Time-Dependent Trajectory)을 결정한다.

궤적은 상태 \\(x_0,x_1,\\ldots,x_T\\)와 제어 입력 \\(u_0,u_1,\\ldots,u_{T-1}\\)의 시퀀스로 표현할 수 있다. 상태에는 3차원 위치, 속도, 자세(Attitude), 각속도(Angular Velocity), 배터리 상태 또는 화물 관련 변수가 포함될 수 있다. 제어 입력은 계획기에 사용되는 UAV 모델의 충실도(Model Fidelity)에 따라 전체 추력, 몸체 토크(Body Torque), 로터 명령(Rotor Command), 또는 상위 수준의 가속도 기준값(Acceleration Reference)을 나타낼 수 있다.

화물 운송은 탑재물 질량(Payload Mass)이 가속 성능, 추력 요구량, 에너지 소비, 기동성(Maneuverability)에 직접적인 영향을 미치기 때문에 최적화 문제를 변화시킨다. 무거운 화물을 운반하는 UAV는 일반적으로 호버링(Hovering)에 더 큰 추력이 필요하며 가속이나 외란 억제(Disturbance Rejection)에 사용할 수 있는 여유 제어 권한(Control Authority)이 감소한다. 따라서 계획기는 무부하 기체의 동적 거동을 가정하기보다 실제 또는 추정된 탑재물 질량을 사용해야 한다.

탑재물은 기체의 질량중심(Center of Mass)과 관성 텐서(Inertia Tensor)도 변화시킬 수 있다. 화물이 기준 질량중심에서 벗어난 위치에 장착되면 회전 동역학(Rotational Dynamics)과 액추에이터 요구량이 달라진다. 부정확한 질량 분포를 기반으로 한 최적화는 수치적으로는 실행 가능해 보이지만 실제로 추종하기 어려운 궤적을 생성할 수 있다. 따라서 탑재물 인식 모델(Payload-Aware Model)은 계획된 움직임과 실제 항공기 응답 사이의 일관성을 향상시킨다.

현수 화물(Suspended Cargo)은 탑재물이 UAV에 대해 흔들릴 수 있기 때문에 추가적인 동역학을 발생시킨다. 단순화된 모델에서는 화물을 케이블로 연결된 진자(Pendulum)로 표현하고 탑재물 각도와 각속도를 시스템 상태에 추가할 수 있다. 급격한 가속, 제동 또는 선회는 진동(Oscillation)을 유발할 수 있다. 궤적 최적화는 화물의 흔들림 각도와 흔들림 속도에 페널티를 부여하면서 허용할 수 없는 탑재물 움직임을 발생시키는 기동을 제한할 수 있다.

목적함수(Objective Function)는 임무 성능과 비행 품질(Flight Quality)을 결합한다. 일반적인 비용 항은 이동 시간, 에너지 소비, 원하는 경로로부터의 편차, 제어 노력(Control Effort), 과도한 가속, 장애물 근접도, 종단 위치 오차(Terminal Position Error)를 억제한다. 화물 특화 비용은 탑재물 흔들림, 급격한 자세 변화, 높은 저크(Jerk), 과도한 하중 계수(Load Factor)를 추가적으로 억제할 수 있다. 가중치 선택은 속도, 효율성, 안전성, 부드러움 사이의 절충 관계를 결정한다.

기체 동역학(Vehicle Dynamics)은 핵심적인 실행 가능성 제약조건(Feasibility Constraint)을 형성한다. 멀티로터(Multirotor) 궤적은 사용 가능한 추력과 토크 한계 안에서 병진 및 회전 운동방정식(Translational and Rotational Equations of Motion)을 만족해야 한다. 단순화된 계획에서는 질점 모델(Point-Mass Model)이나 미분 평탄성(Differential Flatness) 모델을 사용할 수 있으며, 더 높은 충실도의 최적화에서는 완전한 강체 동역학(Full Rigid-Body Dynamics)을 포함할 수 있다. 적절한 모델은 임무 복잡도, 계산 자원, 요구되는 추종 정확도에 따라 결정된다.

미분 평탄성(Differential Flatness)은 위치와 요(Yaw)를 평탄 출력(Flat Output)으로 사용하고 이들로부터 자세, 속도, 가속도, 제어 물리량을 복원할 수 있기 때문에 멀티로터 궤적 생성에 특히 유용하다. 이를 통해 중요한 동역학 관계를 유지하면서 더 적은 수의 변수로 궤적을 최적화할 수 있다. 이후 다항식(Polynomial) 또는 스플라인(Spline) 궤적을 부드러움, 타이밍, 장애물 여유 거리, 액추에이터 실행 가능성을 기준으로 최적화할 수 있다.

최소 스냅 궤적 최적화(Minimum-Snap Trajectory Optimization)는 고기동 멀티로터 움직임과 널리 연관된 방법이다. 구간별 다항식(Piecewise Polynomial Segment)은 웨이포인트(Waypoint)를 연결하면서 스냅(Snap)과 같은 고차 미분값을 최소화하여 부드러운 위치 궤적과 이에 대응하는 제어 명령을 생성한다. 그러나 화물 UAV에서는 수학적으로 부드러운 궤적만으로 탑재물 질량, 에너지 한계, 바람, 현수 화물 진동, 운용 안전 여유를 자동으로 고려할 수 없기 때문에 추가적인 제약조건이 필요할 수 있다.

장애물 회피(Obstacle Avoidance)는 3차원 공간에서의 추론을 요구한다. 건물, 지형, 크레인, 전력 인프라, 나무 및 기타 구조물은 실행 가능한 비행 회랑(Flight Corridor)을 제한할 수 있다. 장애물은 메시(Mesh), 점유 지도(Occupancy Map), 복셀 격자(Voxel Grid), 부호 거리장(Signed-Distance Field), 볼록 영역(Convex Region) 등을 이용하여 표현할 수 있다. 최적화는 기체와 필요한 경우 현수 화물이 환경 형상으로부터 충분한 거리를 유지하도록 여유 거리 제약조건(Clearance Constraint)이나 장애물 페널티를 적용할 수 있다.

볼록 안전 회랑(Convex Safe Corridor)은 전역 경로 계획(Global Path Planning)과 연속 궤적 최적화(Continuous Trajectory Optimization)를 연결하는 실용적인 인터페이스를 제공한다. 먼저 대략적인 계획기가 충돌이 없는 경로를 탐색하고 그 주변에 서로 중첩되는 일련의 볼록 영역을 구성할 수 있다. 이후 다항식 제어점(Polynomial Control Point) 또는 이산 UAV 상태를 이러한 영역 내부에 유지하도록 제한한다. 이를 통해 비볼록 충돌 회피 문제의 일부를 보다 단순한 선형 또는 볼록 제약조건으로 변환할 수 있다.

궤적 타이밍(Trajectory Timing)은 실행 가능성과 밀접하게 결합되어 있다. 동일한 기하학적 경로를 더 빠르게 이동하려면 더 큰 가속도, 추력, 자세 변화가 필요하며, 느린 비행은 일반적으로 동적 요구량을 줄이지만 임무 시간을 증가시키고 맞바람 속에서는 에너지 소비를 증가시킬 수도 있다. 따라서 최적화는 구간 지속시간(Segment Duration)이나 전체 임무 시간을 결정 변수로 사용하여 공간적 경로와 그 경로를 따라 이동하는 속도 프로파일(Speed Profile)을 함께 결정할 수 있다.

에너지 인식 계획(Energy-Aware Planning)은 탑재물이 비행 지속시간(Flight Endurance)을 감소시키기 때문에 화물 UAV에서 특히 중요하다. 배터리 소비는 기체 질량, 공기역학적 조건(Aerodynamic Conditions), 비행 속도, 기동, 바람, 추진 효율(Propulsion Efficiency)에 영향을 받는다. 에너지 모델을 목적함수에 포함하거나 착륙 시 충분한 에너지 예비량(Energy Reserve)을 유지하도록 제약조건으로 사용할 수 있다. 따라서 가장 빠른 궤적이 반드시 에너지 소비가 가장 적거나 운용 여유가 가장 큰 궤적은 아니다.

바람(Wind)은 결정론적 외란(Deterministic Disturbance)과 불확실한 외란(Uncertain Disturbance)을 모두 발생시킨다. 알려진 바람장(Wind Field)은 동역학 모델에 포함하여 계획기가 예상되는 기류를 고려한 헤딩, 속도, 고도 프로파일을 선택하도록 할 수 있다. 강한 맞바람에서는 짧은 기하학적 경로가 비효율적일 수 있으며, 유리한 바람을 활용하기 위해 더 긴 공간 경로가 적절할 수도 있다. 건물이나 지형 주변의 바람 구배(Wind Gradient)는 국소적인 궤적 실행 가능성에 추가적인 영향을 줄 수 있다.

강건한 궤적 계획(Robust Trajectory Planning)은 바람, 탑재물 특성, 상태 추정(State Estimation), 기체 모델의 불확실성을 다룬다. 보수적인 안전 여유를 적용하여 허용 추력을 낮추고 장애물 여유 거리를 증가시키거나 추가적인 배터리 예비량을 확보할 수 있다. 보다 발전된 접근법은 외란 집합(Disturbance Set), 확률적 제약조건(Probabilistic Constraint), 복수의 환경 시나리오를 대상으로 최적화할 수 있다. 이러한 궤적은 실제 운용 조건에서 실행 가능성을 향상시키기 위해 명목 성능(Nominal Performance)의 일부를 희생할 수 있다.

운용 제약조건(Operational Constraint)은 물리적인 충돌 회피를 넘어선다. 화물 임무에는 허용된 비행 회랑, 고도 범위(Altitude Band), 이착륙 구역, 접근 방향, 속도 제한, 비행해서는 안 되는 영역 등이 포함될 수 있다. 이러한 조건은 기하학적 영역, 상태 범위(State Bound), 시간 조건(Temporal Condition), 임무 수준 제약조건(Mission-Level Constraint)으로 표현할 수 있으며, 이를 통해 기체 성능과 운용 요구조건을 동시에 만족하는 궤적을 생성할 수 있다.

이륙과 착륙(Takeoff and Landing)은 지표면 가까이에서 이루어지고 일반적으로 순항 비행보다 안전 여유가 작기 때문에 특별한 처리가 필요하다. 종단 제약조건(Terminal Constraint)은 목적지에서 위치, 속도, 가속도, 요, 접근 방향을 지정할 수 있다. 정밀 화물 배송(Precision Cargo Delivery)의 경우 화물을 방출하기 전에 안정적인 호버링이나 저속 접근을 요구하여 잔류 기체 움직임과 탑재물 흔들림이 허용 범위 내에 있도록 할 수 있다.

순차 볼록 최적화(Sequential Convex Optimization)는 기준 궤적(Nominal Trajectory) 주변에서 비선형 동역학과 충돌 제약조건을 반복적으로 근사하여 화물 UAV 계획 문제를 해결할 수 있다. 각 반복에서는 보다 다루기 쉬운 하위 문제(Subproblem)를 해결한 뒤 기준 궤적을 갱신한다. 신뢰 영역(Trust Region)은 반복 사이의 변화량을 제한하여 국소 근사가 유효하도록 한다. 기하학적 계획기 또는 이전 궤적에서 얻은 좋은 초기값은 수렴 성능을 크게 향상시킬 수 있다.

직접 전사(Direct Transcription)와 직접 콜로케이션(Direct Collocation)은 더 높은 충실도의 계획을 위한 대안적인 방법을 제공한다. 비행 시간 구간 전체의 상태와 제어 입력을 최적화 변수로 설정하고, 인접한 시간 지점 사이에 동역학적 일관성(Dynamic Consistency)을 적용하는 등식 제약조건을 부여한다. 부등식 제약조건은 추력, 속도, 자세, 충돌, 운용 한계를 표현한다. 생성되는 비선형 계획 문제(Nonlinear Program)는 시간적 희소성(Temporal Sparsity)을 활용할 수 있지만 모델 충실도와 시간 해상도가 증가할수록 계산량도 증가한다.

이동 시간 구간 최적화(Receding-Horizon Optimization)를 사용하면 화물 UAV가 비행을 시작한 이후에도 변화하는 상황에 적응할 수 있다. 미리 계산된 전체 궤적을 수정 없이 실행하는 대신 최신 기체 상태, 탑재물 거동, 장애물 정보, 바람 추정값을 이용하여 제한된 미래 구간을 반복적으로 최적화한다. 계산된 해의 초기 일부만 실행한 뒤 다시 계획하므로 궤적 최적화를 모델 예측 제어(Model Predictive Control, MPC)와 자연스럽게 연결할 수 있다.

이동 장애물(Moving Obstacle), 예상하지 못한 바람, 부정확한 탑재물 모델로 인해 기존 궤적이 더 이상 유효하지 않을 때 실시간 재계획(Real-Time Replanning)이 특히 중요하다. 이전 해를 이용한 웜 스타트(Warm Start)는 최적화 시간을 단축하며, GPU 가속(GPU Acceleration)은 충돌 질의, 궤적 평가, 샘플링된 대안, 모델 계산을 병렬화할 수 있다. 빠르게 이동하는 항공기에서는 아무리 우수한 궤적이라도 지나치게 늦게 계산되면 활용 가치가 낮기 때문에 계획 지연시간(Planning Latency)을 제한해야 한다.

계층적 아키텍처(Hierarchical Architecture)를 사용하면 전체 문제를 여러 계획 수준으로 분산할 수 있다. 임무 계획(Mission Planning)은 화물 픽업과 배송 목표를 선택하고, 전역 경로 계획은 전체적인 3차원 비행 회랑을 결정하며, 궤적 최적화는 동역학적으로 실행 가능한 움직임을 생성한다. 이후 비행 제어기(Flight Controller)가 생성된 기준 궤적을 추종한다. 탑재물 추정, 인지(Perception), 위치추정(Localization), 상태 모니터링(Health Monitoring)은 궤적 조정이나 임무 재계획을 유발할 수 있는 정보를 지속적으로 제공한다.

명령을 실제로 실행하기 전에 궤적 검증(Trajectory Validation)이 필요하다. 계획된 움직임은 추력 및 토크 한계, 속도 및 자세 범위, 충돌 여유 거리, 배터리 예비량, 탑재물 안정성, 종단 조건을 기준으로 검사해야 한다. 더 높은 충실도의 시뮬레이션(High-Fidelity Simulation)을 이용하면 단순화된 계획 모델에서 나타나지 않았던 추종 오차나 화물 진동을 확인할 수 있다. 수치적 수렴만으로 실제 운용 가능성이 보장되는 것은 아니므로 실행 중에도 안전 모니터링(Safety Monitoring)을 지속해야 한다.

최적화 기반 화물 UAV 계획(Optimization-Based Cargo UAV Planning)은 경로 형상(Route Geometry), 항공기 동역학, 탑재물 거동, 에너지, 환경 제약조건, 불확실성, 임무 타이밍을 하나의 통합된 궤적 생성 프레임워크(Unified Trajectory-Generation Framework)에서 결합한다. UAV가 어디로 비행할지뿐만 아니라 얼마나 빠르게 이동하고 시간에 따라 제어 입력을 어떻게 변화시킬지를 함께 최적화함으로써 단순히 충돌이 없는 수준을 넘어 동역학적으로 실행 가능하고, 에너지를 고려하며, 부드럽고, 실제 환경에서 반복적인 재계획에 적합한 화물 운송 임무를 생성할 수 있다.
