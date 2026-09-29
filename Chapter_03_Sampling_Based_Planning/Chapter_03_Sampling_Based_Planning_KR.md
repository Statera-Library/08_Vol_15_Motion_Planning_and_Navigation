**Volume 15. Motion Planning and Navigation**

# Chapter 03. Sampling Based Planning

## 03.01. Probabilistic Roadmap PRM Theory and Implementation [w/Code]

![](images/image1.png){width="7.268055555555556in" height="7.268055555555556in"}

확률적 로드맵(Probabilistic Roadmap, PRM)은 충돌 없는 구성 공간(configuration space)의 연결성을 명시적인 기하학적 분할 없이 표현하도록 설계된 샘플링 기반 운동 계획(sampling-based motion planning) 방법이다. 가능한 모든 로봇 구성(configuration)을 전수 탐색하는 대신, PRM은 자유 공간(free space)에서 구성을 샘플링하고 실행 가능한 인접 샘플들을 연결하여 로드맵(roadmap)이라는 그래프(graph)를 구성한다. 이러한 특성으로 인해 PRM은 연속적이고 고차원이거나 기하학적으로 복잡한 구성 공간에서 특히 유용하다.

구성 공간(configuration space)은 개념적으로 자유 공간(free space)과 장애물 공간(obstacle space)으로 구분된다. 구성(configuration)은 이동 로봇의 평면 위치와 방향 또는 매니퓰레이터(manipulator)의 관절각과 같이 로봇의 자세를 완전하게 기술하는 데 필요한 상태를 나타낸다. 각각의 샘플 구성은 충돌 검사기(collision checker)를 통해 검사된다. 장애물과 충돌하거나 제약조건을 위반하는 샘플은 제거되고, 유효한 샘플은 로드맵의 정점(vertex)이 된다. 따라서 생성된 그래프는 연속적인 자유 공간의 연결성을 이산적으로 근사한다.

PRM은 일반적으로 로드맵 구성(roadmap construction)과 질의 처리(query processing)의 두 단계로 구성된다. 구성 단계에서 계획기(planner)는 유효한 구성 집합을 생성하고 지역 계획기(local planner)를 이용하여 인접 구성들을 연결한다. 충돌 없이 연결 가능한 경우 그래프의 간선(edge)이 된다. 질의 단계에서는 시작 구성(start configuration)과 목표 구성(goal configuration)을 적절한 로드맵 정점에 연결한 다음, 그래프 탐색 알고리즘(graph-search algorithm)을 사용하여 이들을 연결하는 구성 시퀀스를 찾는다. 이러한 분리는 동일한 환경에서 다수의 계획 질의를 처리할 때 특히 효과적이다.

샘플링 품질(sampling quality)은 로드맵의 품질에 큰 영향을 미친다. 균일 무작위 샘플링(uniform random sampling)은 단순하고 범용적이지만, 넓은 개방 영역에 지나치게 많은 샘플을 배치하는 반면 좁은 통로(narrow passage)에는 충분한 샘플을 배치하지 못할 수 있다. 대안적인 방법은 장애물 경계, 중앙 영역, 통과하기 어려운 구간 또는 이전 연결에 실패했던 영역에 샘플을 집중시킬 수 있다. 목적은 단순히 샘플 수를 최대화하는 것이 아니라 로봇의 충돌 없는 구성 공간의 위상(topology)과 연결성을 효율적으로 표현하는 정점을 생성하는 것이다.

샘플링 이후 PRM은 어떤 정점들을 이웃(neighbor)으로 간주할 것인지 결정한다. 일반적인 구현에서는 각 정점을 k개의 최근접 이웃(k-nearest neighbors)에 연결하거나 지정된 연결 반경(connection radius) 내부의 모든 정점과 연결한다. 거리는 구성 공간에 적합한 거리 척도(distance metric)를 사용하여 측정한다. 평면 이동에는 유클리드 거리(Euclidean distance)가 충분할 수 있지만, 방향이나 로봇 관절을 포함하는 공간에서는 각도 변수, 스케일링, 관절 범위 및 구성 차원 간의 물리적 의미 차이를 고려하는 거리 척도가 필요하다.

지역 계획기(local planner)는 두 후보 정점을 안전하게 연결할 수 있는지를 판단한다. 기본적인 구현에서는 두 구성 사이를 보간(interpolation)하고 중간 지점에서 충돌 검사(collision checking)를 수행한다. 검사된 모든 구성이 유효한 경우 로드맵에 간선(edge)이 추가된다. 간선 가중치(edge weight)는 기하학적 거리, 이동 시간, 에너지, 여유 거리(clearance) 또는 다른 계획 비용을 나타낼 수 있다. 너무 거친 충돌 검사는 실제로 장애물을 통과하는 간선을 잘못 허용할 수 있으므로 지역 계획에서는 충분히 세밀한 유효성 검사가 필요하다.

로드맵이 구성되면 계획 질의(planning query)는 시작 구성과 목표 구성을 인접한 로드맵 정점에 삽입하거나 임시로 연결한다. 이후 다익스트라(Dijkstra) 또는 A\*(A-Star)와 같은 알고리즘을 사용하여 실행 가능한 경로를 탐색할 수 있다. 생성된 그래프 경로(graph path)는 유효한 지역 운동(local motion)으로 연결된 구성들의 시퀀스로 이루어진다. 이러한 구성은 불필요한 회전이나 우회 경로를 포함할 수 있으므로 실제 구현에서는 하위 운동 제어 구성요소에 전달하기 전에 단축(shortcutting), 평활화(smoothing), 보간(interpolation) 또는 궤적 최적화(trajectory optimization)를 적용하는 경우가 많다.

확률적 완전성(probabilistic completeness)은 PRM을 이해하는 핵심 특성이다. 충분한 여유 공간을 갖는 실행 가능한 경로가 존재하고 적절한 가정 아래에서 샘플링이 무한히 계속된다면, PRM이 연결 가능한 로드맵을 발견하지 못할 확률은 0에 가까워진다. 그러나 이는 유한한 크기의 로드맵이 항상 성공을 보장한다는 의미는 아니다. 샘플 부족, 부적절한 연결 파라미터, 복잡한 좁은 통로, 부정확한 충돌 검사 또는 로봇과 환경을 올바르게 모델링하지 못한 구성 공간 표현으로 인해 계획이 실패할 수 있다.

PRM의 성능은 샘플 수(sample count), 이웃 크기(neighborhood size), 충돌 검사 비용(collision-checking cost), 그래프 탐색 복잡도(graph-search complexity)의 상호작용에 따라 결정된다. 샘플 수를 증가시키면 일반적으로 공간의 커버리지(coverage)가 향상되지만 메모리 사용량과 잠재적인 연결 검사 횟수도 증가한다. 연결 반경을 증가시키면 그래프 연결성이 향상될 수 있지만 충돌 검사 비용과 그래프 밀도도 증가한다. 따라서 효율적인 구현에서는 모든 새로운 구성과 기존의 모든 정점을 비교하기보다 k-d 트리(k-d tree)와 같은 공간 인덱싱 구조(spatial indexing structure) 또는 최근접 이웃 탐색 방법을 사용한다.

충돌 검사(collision checking)는 로드맵 구성 과정에서 가장 큰 계산 비용을 차지하는 경우가 많다. 이동 로봇에서는 로봇 풋프린트(robot footprint)를 점유 지도(occupancy map) 또는 기하학적 장애물과 비교할 수 있다. 매니퓰레이터에서는 보간된 각각의 관절 구성이 순기구학(forward kinematics) 계산과 여러 링크 및 환경 객체에 대한 충돌 검사를 요구할 수 있다. 검증된 간선을 캐싱(caching)하고 명백하게 불가능한 연결을 조기에 제거하며 적응형 충돌 검사 해상도(adaptive collision-checking resolution)를 적용하면 PRM의 기본 구조를 변경하지 않고도 계산 비용을 크게 줄일 수 있다.

PRM은 비용이 높은 로드맵을 한 번 구성한 뒤 반복적으로 재사용할 수 있기 때문에 반복 질의 응용(repeated-query application)에 특히 적합하다. 예를 들어 안정적인 제조 셀(manufacturing cell)에서 동작하는 매니퓰레이터는 주요 장애물이 변하지 않는 상태에서 다양한 시작 및 목표 구성을 반복적으로 받을 수 있다. 로드맵은 자유 공간의 연결성에 관한 재사용 가능한 정보를 저장하므로 이후 질의에서는 주로 시작점과 목표점의 연결 및 상대적으로 저비용인 그래프 탐색만 수행하면 된다. 이는 매번 새로운 탐색 구조를 구축하는 계획기와 구별되는 중요한 특징이다.

동적 환경(dynamic environment)에서는 고전적인 PRM이 구성 공간의 장애물 구조가 로드맵을 재사용할 수 있을 정도로 안정적으로 유지된다고 가정한다는 점을 고려해야 한다. 장애물이 이동하거나 작업 공간이 변경되면 이전에 유효했던 간선이 더 이상 유효하지 않을 수 있다. 실제 시스템에서는 탐색 중 간선을 지연 재검증(lazy revalidation)하거나 영향을 받은 로드맵 영역을 무효화하고, 전체를 다시 구성하는 대신 국부 영역만 재구성할 수 있다. 따라서 PRM은 계층형 내비게이션 구조(layered navigation architecture)에 포함될 수 있지만 빠르게 변화하는 장애물 회피는 일반적으로 지역 계획(local planning)이나 반응형 제어(reactive control)가 담당한다.

구현은 일반적으로 구성 공간 경계(configuration-space bounds), 로봇 제약조건(robot constraints), 상태 유효성 함수(state-validity function), 샘플러(sampler), 거리 척도(distance metric), 지역 연결 절차(local connection procedure)를 정의하는 것에서 시작한다. 계획기는 후보 구성을 반복적으로 샘플링하고 유효하지 않은 상태를 제거한 뒤 인접한 로드맵 정점을 검색하고 후보 간선을 검증하여 성공적인 연결을 가중 그래프(weighted graph)에 저장한다. 질의 실행에서는 시작 및 목표 상태를 검증하고 그래프에 연결한 후 최단 경로 탐색(shortest-path search)을 수행하며, 선택된 구성 시퀀스를 복원하고 필요에 따라 경로를 후처리한다.

실제 로봇 시스템(production robotics)에서 PRM은 완전한 내비게이션 솔루션이 아니라 더 광범위한 운동 계획 파이프라인(motion-planning pipeline)의 하나의 구성요소로 다루어야 한다. 로드맵은 전역 연결 정보(global connectivity information)를 제공하고, 위치 추정(localization)은 현재 로봇 상태를 제공하며, 인지(perception)는 환경 제약조건을 갱신한다. 궤적 생성(trajectory generation)은 기하학적 경로를 실행 가능한 운동으로 변환하고 제어기(controller)는 차량 또는 매니퓰레이터의 동역학을 만족시킨다. 이러한 구조에서 PRM은 이후의 RRT 계열 방법, OMPL 활용, 매니퓰레이터 계획, 키노다이내믹 계획(kinodynamic planning), 실시간 샘플링 계획(real-time sampling planning)으로 이어지는 샘플링 기반 계획의 기초 역할을 수행한다.

## 03.02. RRT Rapidly Exploring Random Tree [w/Code]

![](images/image2.png){width="7.268055555555556in" height="7.268055555555556in"}

신속 탐색 랜덤 트리(Rapidly-Exploring Random Tree, RRT)는 크고 연속적인 구성 공간(configuration space)을 효율적으로 탐색하도록 설계된 샘플링 기반 운동 계획(sampling-based motion planning) 알고리즘이다. 환경 전체에 걸쳐 재사용 가능한 그래프(graph)를 구성하는 로드맵(roadmap) 방식과 달리, RRT는 초기 구성(initial configuration)에서 시작하여 아직 충분히 탐색되지 않은 영역을 향해 트리(tree)를 점진적으로 성장시킨다. 이러한 확장 메커니즘은 밀집하게 방문되지 않은 영역을 자연스럽게 우선 탐색하므로 전수적인 이산화(discretization)가 계산적으로 어려운 고차원 공간에서 효과적이다.

알고리즘은 시작 구성(start configuration)만 포함하는 트리에서 시작한다. 각 반복(iteration)마다 유효한 구성 공간의 경계 내에서 무작위 구성(random configuration)을 샘플링한다. 이후 선택된 거리 척도(distance metric)를 기준으로 기존 트리에서 무작위 샘플과 가장 가까운 노드를 탐색한다. 대부분의 구현에서는 샘플에 직접 연결하는 대신 최근접 노드(nearest node)에서 샘플 방향으로 확장 거리(extension distance) 또는 스텝 크기(step size)라고 불리는 제한된 거리만큼 이동한다.

이 연산은 일반적으로 무작위 샘플(random sample), 최근접 트리 노드(nearest tree node), 새롭게 생성된 구성(new configuration)의 세 요소를 이용하여 설명할 수 있다. 샘플 구성을 q_rand, 가장 가까운 기존 노드를 q_near라고 하면 조향 함수(steering function)는 q_near에서 q_rand 방향으로 새로운 구성 q_new를 생성한다. 이러한 확장 거리를 제한하면 지나치게 긴 간선(edge)이 생성되는 것을 방지하고, 충돌 검사(collision checking)를 수행하면서 트리 성장의 해상도를 제어하여 구성 공간을 점진적으로 탐색할 수 있다.

q_new를 트리에 추가하기 전에 q_near와 q_new 사이의 지역 운동(local motion)이 유효한지 검증해야 한다. 계획기(planner)는 후보 간선을 따라 구성을 보간(interpolation)하고 장애물 및 기타 상태 제약조건에 대한 충돌 검사를 수행한다. 전체 지역 운동이 유효하면 q_new는 새로운 트리 정점(tree vertex)이 되고 두 구성을 연결하는 경로는 간선(edge)이 된다. 충돌이 발생하면 구현 방법에 따라 해당 확장을 거부하거나 적절한 유효 지점에서 확장을 종료한다.

RRT의 특징적인 탐색 동작은 최근접 이웃 확장(nearest-neighbor expansion)에 의해 생성되는 보로노이 편향(Voronoi bias)에서 비롯된다. 구성 공간에서 넓고 아직 탐색되지 않은 영역과 연관된 트리 정점은 무작위 샘플의 최근접 정점으로 선택될 가능성이 높다. 결과적으로 트리의 가지(branch)는 이미 밀집된 영역을 반복적으로 확장하기보다 탐색되지 않은 외부 영역으로 성장하는 경향을 갖는다. 이 특성을 통해 RRT는 명시적인 탐색 휴리스틱(exploration heuristic)이나 자유 공간 전체 표현 없이도 넓은 공간을 빠르게 탐색할 수 있다.

목표 지향 계획(goal-directed planning)은 목표 편향(goal biasing)을 통해 개선할 수 있다. 모든 샘플을 균일하게 무작위 선택하는 대신 계획기는 일정한 확률로 목표 구성(goal configuration) 자체를 선택하거나 목표 주변 영역에서 샘플링할 수 있다. 적절한 수준의 목표 편향은 장애물을 우회하는 데 필요한 탐색 능력을 유지하면서 기존 가지가 목적지 방향으로 성장할 가능성을 높인다. 반면 지나치게 높은 목표 편향은 대체 경로를 발견하기보다 차단된 방향으로 반복적인 확장을 시도하게 만들어 오히려 성능을 저하시킬 수 있다.

트리가 성장함에 따라 최근접 이웃 탐색(nearest-neighbor search)의 중요성도 증가한다. 단순한 구현에서는 각각의 무작위 샘플을 모든 기존 노드와 비교할 수 있지만 트리가 커질수록 계산 비용이 크게 증가한다. 적합한 구성 공간에서는 k-d 트리(k-d tree)와 같은 공간 인덱싱 구조(spatial indexing structure)를 사용하여 최근접 이웃 질의를 가속할 수 있다. 그러나 고차원 계획에서는 이러한 구조의 효율성이 차원 수, 거리 척도, 구성 표현 및 생성된 상태의 분포에 크게 좌우된다.

RRT에서 사용하는 거리 척도(distance metric)는 로봇 구성의 물리적 의미를 반영해야 한다. 단순한 평면 병진 계획에는 유클리드 거리(Euclidean distance)가 적합할 수 있지만 이동 로봇은 위치와 방향을 결합한 거리 척도가 필요할 수 있다. 매니퓰레이터(manipulator)는 여러 관절 변수에 대한 거리를 고려해야 하며, 단위나 범위가 서로 다른 구성 차원에는 정규화(normalization) 또는 가중치 부여(weighting)가 필요할 수 있다. 부적절한 거리 척도는 수학적으로는 가까워 보이지만 실제 로봇 운동에는 적합하지 않은 이웃을 선택하게 만들 수 있다.

스텝 크기(step size) 역시 핵심적인 파라미터(parameter)이다. 큰 스텝 크기는 개방된 영역에서 트리를 빠르게 확장할 수 있지만 확장 경로가 장애물과 충돌하거나 중요한 기하학적 구조를 놓칠 가능성을 증가시킨다. 작은 스텝 크기는 장애물과 좁은 통로(narrow passage) 주변을 더욱 세밀하게 탐색할 수 있지만 더 많은 정점을 생성하고 계획 시간을 증가시킨다. 실제 구현에서는 로봇의 기하 구조, 환경 규모, 충돌 검사 해상도 및 예상되는 구성 공간의 복잡성에 따라 확장 거리를 결정한다.

PRM과 마찬가지로 RRT는 적절한 가정 아래에서 확률적 완전성(probabilistic completeness)을 갖는다. 실행 가능한 해가 존재하고 충분한 시간 동안 샘플링을 계속한다면 계획기가 해를 발견할 확률은 1에 가까워진다. 그러나 기본 RRT 알고리즘은 발견된 경로가 최적임을 보장하지 않는다. RRT의 주요 목적은 실행 가능한 경로(feasible path)를 신속하게 발견하는 것이다. 따라서 생성된 궤적은 트리 확장의 확률적 과정으로 인해 불필요한 회전, 긴 우회 경로 또는 불규칙한 구간을 포함할 수 있다.

트리가 목표에 도달하거나 지정된 목표 영역(goal region)에 진입하면 목표 측 노드에서 루트(root)까지 부모 관계(parent relationship)를 역으로 따라가면서 해 경로(solution path)를 복원할 수 있다. 이 시퀀스를 다시 역순으로 배열하면 초기 구성에서 목적지까지의 경로가 생성된다. 원시 RRT 경로(raw RRT path)는 일반적으로 불규칙하므로 실제 시스템에서는 불필요한 웨이포인트(waypoint)를 줄이고 로봇 동역학 및 제어기(controller)와의 호환성을 높이기 위해 단축(shortcutting), 경로 평활화(path smoothing), 보간(interpolation) 또는 후속 궤적 최적화(trajectory optimization)를 적용하는 경우가 많다.

RRT는 특히 단일 질의 계획(single-query planning) 문제에서 높은 가치를 갖는다. PRM은 향후 여러 질의를 지원할 수 있는 로드맵을 구성하기 위해 사전에 계산 자원을 투입하는 반면, RRT는 현재의 시작점-목표점(start-to-goal) 문제를 위해 탐색 구조를 구축한다. 따라서 환경이나 질의가 자주 변경되거나 재사용 가능한 전처리(preprocessing)의 이점이 적은 경우, 또는 구성 공간이 매우 복잡하여 포괄적인 로드맵을 구축하는 비용이 큰 경우에 효과적으로 사용할 수 있다.

기본 RRT는 기하학적(geometric) 방법이며 상세한 로봇 동역학(robot dynamics)을 본질적으로 강제하지 않는다. 미래 상태가 제어 입력(control input), 가속도, 속도, 조향 제한 또는 비홀로노믹 제약(nonholonomic constraint)에 크게 의존하는 시스템에서는 확장 연산(extension operation)에 적절한 운동 모델(motion model)을 포함해야 한다. 이러한 접근은 기하학적 구성을 단순히 보간하는 대신 후보 제어 입력을 시스템 동역학을 통해 전파하는 키노다이내믹 RRT(kinodynamic RRT)로 발전한다. 이는 차동 구동 로봇, 차량, 무인항공기(UAV) 및 동적 제약을 갖는 플랫폼에서 중요하다.

RRT는 여러 주요 샘플링 기반 계획기(sampling-based planner)의 개념적 기반이기도 하다. RRT-Connect는 탐색 구조를 적극적으로 성장시키고 양방향 탐색(bidirectional exploration)을 활용하여 실행 가능한 해를 빠르게 얻을 수 있다. RRT\*(RRT-Star)는 부모 선택(parent selection)과 재배선(rewiring)을 도입하여 경로 품질을 점진적으로 향상시킨다. Informed RRT\*는 초기 해가 발견된 이후 현재 경로를 개선할 가능성이 있는 영역으로 샘플링을 제한한다. 이러한 발전형 알고리즘은 속도 또는 최적성을 개선하면서 점진적인 트리 탐색이라는 RRT의 핵심 원리를 유지한다.

로봇 운동 계획 아키텍처(robotics motion-planning architecture)에서 RRT는 주변 구성요소로부터 로봇 상태(robot state), 목표(goal), 구성 공간 제약조건(configuration-space constraints), 충돌 정보(collision information), 운동 모델 가정(motion-model assumptions)을 입력받는다. RRT의 출력은 일반적으로 액추에이터(actuator)에 직접 전달되는 명령이 아니라 충돌 없는 기하학적 경로(collision-free geometric path) 또는 상태 시퀀스(state sequence)이다. 이후 궤적 생성(trajectory generation)이 시간 및 동적 실행 가능성을 부여하고 제어기(controller)가 생성된 운동을 실행한다. 따라서 RRT는 연속 공간 탐색(continuous-space exploration)과 실행 가능한 로봇 내비게이션 또는 조작(manipulation)을 연결하는 역할을 수행한다.

## 03.03. RRT Star Asymptotically Optimal Planning [w/Code]

![](images/image3.png){width="7.268055555555556in" height="7.268055555555556in"}

RRT\*(Rapidly-Exploring Random Tree Star)는 신속 탐색 랜덤 트리(Rapidly-Exploring Random Tree, RRT) 프레임워크를 실행 가능한 운동 계획(feasible motion planning)에서 점근적 최적 계획(asymptotically optimal planning)으로 확장한 알고리즘이다. 기본 RRT와 마찬가지로 무작위 샘플링(random sampling)과 충돌 검증된 트리 확장(collision-checked tree expansion)을 통해 연속적인 구성 공간(configuration space)을 점진적으로 탐색한다. 가장 중요한 차이점은 새롭게 생성된 구성을 단순히 가장 가까운 기존 노드에 연결하지 않는다는 것이다. RRT\*는 주변의 대안적인 연결을 평가하고 트리를 지속적으로 재구성하여 탐색이 진행될수록 샘플 상태에 도달하는 비용을 감소시킨다.

알고리즘은 시작 구성(start configuration)을 탐색 트리(search tree)의 루트(root)로 설정하면서 시작한다. 무작위 구성 q_rand를 샘플링하고 가장 가까운 기존 노드 q_nearest를 찾은 다음, 조향 연산(steering operation)을 통해 후보 구성 q_new를 생성한다. q_nearest에서 q_new까지의 지역 운동(local motion)은 충돌 및 제약조건 위반 여부를 검사한다. 확장이 유효하면 RRT\*는 q_nearest를 즉시 영구적인 부모(parent)로 지정하지 않고 q_new 주변의 이웃 영역(neighborhood)을 추가로 조사한다.

이러한 이웃 탐색(neighborhood search)은 RRT\*와 기존 RRT를 구분하는 핵심 요소이다. 계획기(planner)는 q_new로부터의 거리가 연결 반경(connection radius) 내부에 있거나 최근접 이웃 기준(nearest-neighbor criterion)을 만족하는 주변 정점들을 수집한다. 각각의 이웃 정점은 새로운 구성에 대한 잠재적인 부모가 된다. RRT\*는 루트에서 각 후보 부모까지의 누적 비용과 해당 부모에서 q_new까지의 지역 연결 비용(local connection cost)을 함께 평가하며, 충돌 없는 연결이 불가능한 후보는 제외한다.

부모 선택(parent selection) 과정에서는 q_new까지의 전체 비용이 가장 낮아지는 유효한 이웃 노드를 선택한다. 따라서 기하학적으로 가장 가까운 노드가 반드시 부모가 되는 것은 아니다. 조금 더 멀리 떨어진 정점이라도 이미 훨씬 저비용의 트리 가지(tree branch)에 포함되어 있다면 더 좋은 경로를 제공할 수 있다. 이러한 비용 인식 연결(cost-aware attachment) 메커니즘을 통해 RRT\*는 실행 가능한 경로를 발견한 이후의 후처리(post-processing)에만 의존하지 않고 트리를 구성하는 과정 자체에서 경로 품질을 향상시킨다.

q_new를 삽입한 이후 RRT\*는 두 번째 핵심 연산인 재배선(rewiring)을 수행한다. 주변 정점에 대해 q_new를 경유하여 도달하는 것이 기존 루트 경로보다 비용을 줄일 수 있는지를 다시 평가한다. q_new를 통한 충돌 없는 연결이 더 낮은 비용을 제공하면 해당 정점은 부모 노드를 변경한다. 이때 영향을 받는 자손 노드(descendant)의 비용도 일관되게 갱신해야 한다. 반복적인 재배선을 통해 초기의 불규칙한 랜덤 트리는 점차 효율적인 경로를 포함하는 구조로 변화한다.

최적화 목적(optimization objective)은 비용 함수(cost function)를 통해 표현된다. 단순한 기하학적 계획에서는 간선 비용(edge cost)을 유클리드 거리(Euclidean distance)로 정의하고 최소 경로 길이를 목적 함수로 사용할 수 있다. 보다 현실적인 로봇 응용에서는 이동 시간(travel time), 에너지 소비(energy consumption), 장애물 여유 거리(obstacle clearance), 지형 난이도(terrain difficulty), 제어 노력(control effort) 또는 여러 기준의 조합을 사용할 수 있다. 따라서 최적성(optimality)의 의미는 선택한 목적 함수에 직접적으로 의존하며, 부적절한 비용 함수는 수학적으로는 최적이지만 실제 운용에는 바람직하지 않은 경로를 생성할 수 있다.

이웃 반경(neighborhood radius)은 계산 효율성과 이론적 특성 모두에서 중요한 역할을 한다. 이웃 영역이 지나치게 작으면 유용한 대체 부모와 재배선 기회를 놓칠 수 있어 알고리즘이 일반적인 RRT와 유사하게 동작할 수 있다. 반대로 지나치게 크면 새로운 노드를 삽입할 때마다 많은 충돌 검사와 비용 비교가 필요하다. RRT\* 이론에서는 일반적으로 정점 수에 따라 변화하는 이웃 크기를 사용하여 트리가 조밀해질수록 불필요한 비교를 줄이면서도 충분한 연결성을 유지하도록 한다.

RRT\*는 무작위 샘플링(random sampling)과 보로노이 편향(Voronoi bias)에 기반한 탐색 특성을 유지한다. 새로운 샘플은 트리 가지가 아직 탐색되지 않은 영역으로 진입하도록 유도하며, 부모 최적화(parent optimization)와 재배선은 이미 발견된 트리 영역의 구조를 개선한다. 이러한 조합은 탐색(exploration)과 지역적 구조 개선(local structural improvement)을 분리한다. 즉 샘플링은 공간의 탐색 범위를 확장하고 최적화는 연결 구조를 재구성한다. 따라서 계획기는 최초의 실행 가능한 경로가 목표에 도달한 이후에도 탐색을 계속하면서 기존 해를 개선할 수 있다.

점근적 최적성(asymptotic optimality)은 RRT\*의 핵심적인 이론적 특성이다. 적절한 가정 아래에서 샘플 수가 무한대로 증가하면 트리에 포함된 최상의 해 비용은 확률 1로 최적해의 비용에 수렴한다. 그러나 이러한 특성을 유한 시간 내의 최적성 보장으로 해석해서는 안 된다. 실제 구현에서는 제한된 계산 예산(computation budget) 이후 알고리즘을 종료해야 하므로 반환되는 경로는 일반적으로 유한 시간 내에서 최적임이 증명된 결과가 아니라 그 시점까지 발견된 최상의 해(best-so-far solution)이다.

이러한 애니타임 특성(anytime behavior)은 로봇 시스템에서 유용하다. RRT\*는 먼저 실행 가능한 해를 얻은 다음 계산 시간이 허용되는 동안 샘플링을 계속할 수 있다. 성공적인 부모 변경이나 재배선 연산이 수행될 때마다 현재의 최상 비용이 감소할 수 있다. 따라서 계획 시스템은 계산 시간과 해의 품질 사이에서 절충할 수 있다. 빠른 응답이 필요한 응용에서는 허용 가능한 경로를 찾은 직후 종료할 수 있으며, 시간 제약이 상대적으로 적은 작업에서는 궤적 실행 전에 추가적인 최적화를 수행할 수 있다.

RRT\*가 처음 발견한 실행 가능한 해는 여전히 기존 RRT 경로와 유사하게 불필요한 굴곡이나 우회 경로를 포함할 수 있다. 그러나 샘플링이 계속되면서 대체 연결과 재배선을 통해 목적 함수에 따라 경로가 점진적으로 짧아지거나 개선된다. 경로 추출(path extraction)은 선택된 목표 노드에서 루트까지 부모 연결(parent link)을 역으로 추적하여 수행한다. 점근적인 그래프 최적성(asymptotic graph optimality)이 자동으로 부드럽거나 동역학적으로 실행 가능한 운동을 보장하는 것은 아니므로 추가적인 단축(shortcutting), 평활화(smoothing) 또는 궤적 최적화(trajectory optimization)를 적용할 수 있다.

충돌 검사(collision checking)는 여전히 주요 계산 비용 중 하나이다. 모든 후보 확장(candidate extension), 대체 부모 연결(alternative parent connection), 잠재적인 재배선 간선은 장애물에 대한 유효성 검사를 요구할 수 있다. 따라서 효율적인 구현에서는 공간 최근접 이웃 구조(spatial nearest-neighbor structure), 선택적 이웃 탐색(selective neighborhood search), 유효성 정보 캐싱(cached validity information), 적절한 충돌 검사 해상도를 함께 사용한다. 특히 매니퓰레이터(manipulator)나 기하학적으로 복잡한 로봇에서는 추가적인 재배선의 이점과 반복적인 연결 검증 비용 사이의 균형이 중요하다.

RRT\*는 로봇 동역학(robot dynamics)이 조향 및 상태 전파 메커니즘에 명시적으로 포함되지 않는 한 기본적으로 기하학적 계획기(geometric planner)이다. 이동 로봇, 매니퓰레이터, 무인항공기(UAV) 또는 기타 제약 시스템에서는 기하학적인 경로 최적성이 동역학적으로 실행 가능한 궤적 최적성을 의미하지 않을 수 있다. 속도, 가속도, 조향, 비홀로노믹 제약(nonholonomic constraint), 제어 한계(control limit)는 키노다이내믹 확장(kinodynamic extension) 또는 후속 궤적 생성을 필요로 할 수 있다. 따라서 RRT\*는 일반적으로 고품질의 충돌 없는 경로를 제공하고 이후 단계에서 이를 실행 가능한 운동으로 변환한다.

여러 고급 계획기(advanced planner)는 RRT\*의 원리를 기반으로 발전하였다. Informed RRT\*는 최초의 해를 발견한 이후 현재 최상의 경로를 개선할 가능성이 있는 구성 공간의 부분집합으로 샘플링을 제한하여 관련성이 낮은 영역에 사용되는 계산량을 줄인다. 양방향(bidirectional) 및 휴리스틱(heuristic) 변형은 초기 해 발견이나 수렴 속도를 향상시키는 것을 목표로 한다. 이러한 방법은 특정 계획 조건에서 탐색 효율성을 높이면서 샘플링, 이웃 기반 부모 선택, 비용 전파(cost propagation), 재배선이라는 기본 개념을 유지한다.

샘플링 기반 계획(sampling-based planning)의 전체 구조에서 RRT\*는 기본 RRT 이후에 위치하며 Informed RRT\*, 양방향 RRT-Connect, OMPL 기반 구현, 매니퓰레이터 계획(manipulator planning), 키노다이내믹 확장(kinodynamic extension)으로 이어지는 기반을 제공한다. RRT\*의 핵심적인 개념적 기여는 단순히 실행 가능한 경로를 빠르게 발견하는 것에서 벗어나 탐색 트리를 최적 경로에 가까워지도록 지속적으로 개선하는 것으로 전환한 데 있으며, 이는 현대 로봇 운동 계획에서 사용되는 최적화 지향 샘플링 계획기(optimization-oriented sampling planner)의 중요한 기반을 형성한다.

## 03.04. Informed RRT Star Ellipsoidal Sampling [w/Code]

![](images/image4.png){width="7.268055555555556in" height="7.268055555555556in"}

Informed RRT\*는 초기 실행 가능 경로(feasible path)가 발견된 이후 최적해(optimal solution)를 향한 수렴 속도를 높이도록 설계된 RRT\*의 확장 알고리즘이다. 표준 RRT\*는 많은 영역이 현재 해를 개선할 가능성이 없더라도 전체 구성 공간(configuration space)에서 계속 샘플링한다. Informed RRT\*는 이러한 비효율성을 해결하기 위해 이후의 샘플링을 시작점과 목표점 사이에서 더 낮은 비용의 경로를 생성할 가능성이 있는 상태들로 구성된 기하학적으로 정의된 부분집합(subset)으로 제한한다.

최초의 실행 가능한 해가 발견되기 전까지 Informed RRT\*는 기본적으로 기존 RRT\*와 동일하게 동작한다. 허용 가능한 구성 공간에서 무작위 구성(random configuration)을 샘플링하고, 최근접 이웃 선택(nearest-neighbor selection)과 조향(steering)을 통해 트리를 확장하며, 충돌 없는 노드를 삽입한다. 부모 선택(parent selection)과 재배선(rewiring)을 통해 RRT\*와 동일하게 트리 구조를 개선한다. 아직 의미 있는 정보 기반 샘플링 영역(informed sampling region)을 정의할 수 있는 해 비용의 상한(upper bound)이 존재하지 않기 때문에 이러한 제한 없는 탐색이 필요하다.

실행 가능한 경로가 목표점에 도달하면 해당 경로의 비용은 최적해 비용의 상한인 c_best를 제공한다. 경로 길이 최적화(path-length optimization)의 경우 이론적인 하한(lower bound)은 시작 구성과 목표 구성 사이의 직선거리 c_min이다. c_best보다 짧은 경로에 포함될 가능성이 있는 모든 상태는 시작점에서 해당 상태까지의 최소 가능 거리와 해당 상태에서 목표점까지의 최소 가능 거리의 합이 c_best보다 작아야 한다. 이러한 관찰을 통해 정보 기반 부분집합(informed subset)을 정의할 수 있다.

유클리드 경로 길이 계획(Euclidean path-length planning)에서 정보 기반 부분집합은 장형 초타원체(prolate hyperspheroid)의 기하학적 형태를 가지며, 2차원에서는 타원(ellipse), 3차원에서는 타원체(ellipsoid)로 나타난다. 시작 구성과 목표 구성은 두 초점(focal point)을 형성한다. 주요 횡직경(major transverse diameter)은 c_best에 의해 결정되며, 최소 가능 직경은 c_min과 관련된다. 이 영역 외부의 모든 상태는 장애물이 전혀 없더라도 해당 상태를 통과하는 경로가 현재 최상의 해를 개선할 수 없으므로 탐색에서 제외할 수 있다.

따라서 타원체 샘플링(ellipsoidal sampling)은 최초의 해가 발견된 이후 탐색 방식을 변화시킨다. 전체 계획 영역의 관련성이 낮은 부분에서 반복적으로 구성을 추출하는 대신 정보 기반 영역 내부에서 직접 샘플을 생성한다. 더 좋은 경로가 발견되어 c_best가 감소하면 타원체는 시작점과 목표점을 연결하는 통로 주변으로 축소된다. 그 결과 계획기는 추가적인 개선 가능성이 남아 있는 영역에 점점 더 많은 계산 자원을 집중하게 된다.

정보 기반 타원체 내부에서 균일하게 샘플링하려면 단순히 축 정렬 경계 상자(axis-aligned bounding box)에서 점을 생성하는 것 이상의 과정이 필요하다. 일반적인 구현에서는 먼저 단위 n차원 구(unit n-dimensional ball)에서 균일하게 샘플을 생성한다. 이후 정보 기반 초타원체의 주 반경(principal radius)에 따라 샘플의 크기를 조정하고, 장축(major axis)이 시작점에서 목표점으로 향하는 벡터와 정렬되도록 회전시킨 다음 두 점 사이의 중점(midpoint)으로 평행 이동한다. 이를 통해 전체 구성 공간에서 비효율적인 거부 샘플링(rejection sampling)을 수행하지 않고 원하는 부분집합에서 직접 샘플을 생성할 수 있다.

이러한 변환은 c_best와 c_min 사이의 관계에 따라 결정된다. 시작점-목표점 방향의 주 반경은 c_best에 비례하며, 나머지 반경은 c_best의 제곱과 c_min의 제곱 차이에 대한 제곱근과 관련된다. 현재 해의 품질이 낮으면 정보 기반 영역은 상대적으로 넓게 유지된다. 반대로 해가 이론적인 하한에 가까워질수록 영역은 점차 좁아지며 추가적인 최적화와 가장 관련성이 높은 구성 주변으로 탐색을 집중시킨다.

정보 기반 샘플 q_rand가 생성된 이후의 계획 과정은 기본적인 RRT\* 메커니즘을 따른다. 계획기는 인접한 트리 정점(tree vertex)을 찾고 샘플 방향으로 조향한 다음 지역 연결(local connection)의 유효성을 검사하며, 운동이 충돌하지 않으면 q_new를 생성한다. 이후 주변 정점들을 조사하여 더 낮은 비용을 제공하는 부모를 선택하고, q_new를 통해 기존 정점의 누적 비용을 줄일 수 있는 경우 재배선을 수행한다. 따라서 정보 기반 샘플링은 RRT\*의 최적화 과정을 대체하는 것이 아니라 탐색이 수행되는 위치를 변경한다.

가장 큰 이점은 유용한 해가 존재하는 영역에 비해 전체 구성 공간이 매우 큰 경우에 나타난다. 표준 RRT\*는 시작점과 목표점을 연결하는 통로에서 멀리 떨어진 영역에서도 계속 샘플을 생성할 수 있으며, 이러한 샘플은 해를 개선하지 못하면서 최근접 이웃 질의, 충돌 검사, 메모리 및 재배선 연산을 소비한다. Informed RRT\*는 이러한 불필요한 계산을 점진적으로 제거한다. 전체 구성 공간 중 최적화와 관련된 영역의 비율이 매우 작아질 수 있는 고차원 계획(high-dimensional planning)에서는 이러한 효과가 특히 중요해질 수 있다.

이 방법은 RRT\*의 애니타임 특성(anytime characteristic)을 유지한다. 실행 가능한 경로는 발견되는 즉시 반환할 수 있으며, 추가적인 계산 시간을 이용하여 경로 비용을 계속 개선할 수 있다. 새롭게 발견된 저비용 해는 c_best를 감소시키고 이에 따라 정보 기반 샘플링 영역도 축소시킨다. 이는 더 좋은 해가 더 작은 탐색 영역을 만들고, 더 작은 탐색 영역이 샘플링 효율성을 높이며, 향상된 효율성이 주어진 계획 시간 내에서 추가적인 개선으로 이어질 수 있는 피드백 과정(feedback process)을 형성한다.

정보 기반 샘플링이 장애물로 인해 발생하는 어려움을 제거하는 것은 아니다. 타원체는 최적화 목적(optimization objective)에 따라 현재 해를 이론적으로 개선할 가능성이 있는 상태를 나타내지만, 해당 영역의 일부는 여전히 장애물 내부에 위치하거나 서로 연결되지 않은 자유 공간 구성요소(disconnected free-space component)에 포함될 수 있다. 따라서 충돌 검사(collision checking)는 여전히 필수적이다. 좁은 통로(narrow passage)나 복잡한 장애물 배치를 포함하는 환경에서는 작은 정보 기반 영역에서도 더 우수하고 위상적으로 유효한 경로를 발견하기 위해 상당한 탐색이 필요할 수 있다.

Informed RRT\*의 효과는 최적화 목적에도 영향을 받는다. 일반적인 타원체 공식은 시작점과 목표점으로부터의 거리를 이용하여 기하학적 하한을 표현할 수 있기 때문에 유클리드 경로 길이 최소화(Euclidean path-length minimization)에서 자연스럽게 도출된다. 에너지, 위험도, 지형, 동적 제약조건 또는 여러 가중 비용을 포함하는 목적 함수는 동일한 단순 타원체 부분집합을 생성하지 않을 수 있다. 따라서 보다 일반적인 정보 기반 계획기(informed planner)에서는 허용 가능한 비용 휴리스틱(admissible cost heuristic) 또는 현재 해를 개선할 가능성이 있는 상태를 식별하는 다른 방법이 필요하다.

샘플링 영역을 제한한 이후에도 파라미터 선택(parameter selection)은 중요하다. 조향 거리(steering distance)는 트리가 새롭게 샘플링된 영역을 얼마나 빠르게 따라갈 수 있는지에 영향을 주며, 이웃 반경(neighborhood radius)은 부모 선택과 재배선의 효율성에 영향을 미친다. 충돌 검사 해상도(collision-checking resolution)는 간선 검증의 신뢰성과 계산 비용을 결정한다. 트리가 성장함에 따라 최근접 이웃 데이터 구조(nearest-neighbor data structure)의 중요성도 증가한다. 정보 기반 샘플링은 불필요한 탐색을 줄이지만 대규모 탐색 트리를 유지하고 최적화하는 데 필요한 계산 자체를 제거하지는 않는다.

기본 RRT와 비교하면 Informed RRT\*는 점근적 최적화(asymptotic optimization)와 집중된 샘플링(focused sampling)을 모두 도입한다. RRT\*와 비교했을 때 핵심적인 혁신은 다른 재배선 규칙을 사용하는 것이 아니라 현재 최상의 해를 직접 활용하여 이후의 샘플을 유도한다는 점이다. RRT\*가 새롭게 샘플링된 구성이 트리를 개선할 수 있는지를 판단한다면, Informed RRT\*는 상당한 계산 자원을 투입하기 전에 특정 구성이 현재 해를 개선할 가능성이 존재하는지를 추가적으로 판단한다.

샘플링 기반 계획(sampling-based planning)의 전체 구조에서 Informed RRT\*는 RRT와 RRT\* 이후에 위치하며, 이후 양방향 RRT-Connect, OMPL 활용, 매니퓰레이터 계획(manipulator planning), 키노다이내믹 RRT(kinodynamic RRT)로 이어진다. Informed RRT\*의 핵심적인 기여는 현재 해의 품질을 유용한 샘플이 존재할 수 있는 위치에 대한 기하학적 정보로 변환하는 데 있다. 이를 통해 계획기는 광범위한 탐색(broad exploration)에서 점점 더 집중된 최적화(focused optimization)로 전환하면서도 RRT\*의 확률적 특성과 점근적 최적성(asymptotic optimality)을 유지할 수 있다.

## 03.05. Bi Directional RRT Connect Planning [w/Code]

![](images/image5.png){width="7.268055555555556in" height="7.268055555555556in"}

양방향 RRT-Connect(Bidirectional RRT-Connect)는 두 개의 탐색 트리(search tree)를 동시에 성장시켜 실행 가능한 경로(feasible path)를 빠르게 찾도록 설계된 샘플링 기반 운동 계획(sampling-based motion planning) 방법이다. 하나의 트리는 시작 구성(start configuration)에서 시작하고 다른 트리는 목표 구성(goal configuration)에서 시작한다. 하나의 트리가 두 지점 사이의 전체 거리를 탐색하도록 하는 대신, 두 트리가 충돌 없는 구성 공간(collision-free configuration space)을 통해 확장되면서 서로 만나도록 시도한다. 이러한 양방향 전략(bidirectional strategy)은 많은 계획 문제에서 필요한 탐색량을 크게 줄일 수 있다.

이 방법은 신속 탐색 랜덤 트리(Rapidly-Exploring Random Tree, RRT)의 기본 메커니즘을 기반으로 한다. 구성 공간(configuration space)에서 무작위 구성 q_rand를 샘플링하고, 하나의 활성 트리(active tree)에서 가장 가까운 노드 q_near를 찾는다. 조향 연산(steering operation)은 샘플 방향으로 새로운 구성을 생성한다. 지역 운동(local motion)이 충돌하지 않으면 새로운 구성을 트리에 삽입한다. 이후 RRT-Connect의 특징적인 절차에서는 반대쪽 트리를 새롭게 도달한 구성 방향으로 확장하여 연결을 시도한다.

RRT-Connect는 일반적으로 확장(EXTEND) 연산과 연결(CONNECT) 연산을 구분한다. EXTEND는 가장 가까운 트리 노드에서 목표 구성 방향으로 제한된 확장을 수행하며, 일반적으로 사전에 정의된 하나의 스텝(step)만큼 이동한다. 그 결과는 진행이 불가능한 경우 차단(Trapped), 부분적으로 진행한 경우 전진(Advanced), 목표에 도달한 경우 도달(Reached) 상태로 표현할 수 있다. 이러한 상태는 트리 확장을 제어하고 추가적인 연결 시도를 계속할지를 결정하는 간결한 메커니즘을 제공한다.

CONNECT 연산은 한 번의 성공적인 스텝 이후 중단하지 않고 동일한 목표를 향해 EXTEND를 반복적으로 적용한다. 각각의 새로운 구간이 유효하게 유지되는 동안 트리는 목표 구성 방향으로 계속 전진한다. 이러한 적극적인 확장(aggressive extension) 동작은 RRT-Connect가 기본 RRT보다 훨씬 빠르게 해를 발견하는 주요 이유 중 하나이다. 비교적 개방된 영역에서는 한 번의 연결 시도만으로 트리 가지(tree branch)가 구성 공간의 상당한 거리를 이동할 수 있다.

일반적인 반복 과정에서는 먼저 하나의 트리를 q_rand 방향으로 확장한다. 이 연산을 통해 새로운 구성 q_new가 성공적으로 생성되거나 도달되면 계획기(planner)는 두 번째 트리에 q_new를 향한 CONNECT 연산을 수행하도록 한다. 두 번째 트리가 충돌 없이 해당 구성에 도달하면 두 트리가 연결되고 시작점과 목표점 사이의 완전한 경로가 생성된다. 연결에 실패하더라도 두 트리는 이후 반복에서 계속 사용되며 자유 공간(free space)의 다른 영역을 탐색한다.

일반적으로 두 트리의 역할은 각 반복 이후 서로 교환된다. 이전에 무작위 샘플 방향으로 확장했던 트리는 연결 대상(connection target)이 되고, 다른 트리는 활성 확장 트리(active expansion tree)가 된다. 이러한 역할 교환은 계획 문제의 양쪽 끝에서 균형 잡힌 탐색을 수행하도록 한다. 일부 구현에서는 트리 크기 또는 다른 휴리스틱(heuristic)을 기준으로 확장할 트리를 선택할 수도 있지만, 기본 알고리즘에서는 단순한 교대 성장(alternating growth)만으로도 충분하다.

최근접 이웃 탐색(nearest-neighbor search)은 모든 확장에서 성장의 기준이 되는 적절한 노드를 찾아야 하기 때문에 여전히 중요한 계산 요소이다. 작은 트리에서는 선형 탐색(linear search)으로 충분할 수 있지만 대규모 계획 문제에서는 k-d 트리(k-d tree)와 같은 구조를 활용할 수 있다. 선택한 거리 척도(distance metric)는 특히 병진 이동, 회전, 여러 관절 변수 또는 서로 다른 물리적 의미를 갖는 차원을 포함하는 상태에서 로봇 구성을 적절하게 표현해야 한다.

충돌 검사(collision checking)는 각각의 트리 확장이 물리적으로 유효한지를 결정한다. 지역 계획기(local planner)는 구성 사이를 보간(interpolation)하고 중간 상태가 장애물 및 구성 제약조건(configuration constraint)을 위반하는지 검사한다. CONNECT는 여러 번의 연속적인 확장을 수행할 수 있으므로 효율적인 충돌 검사가 특히 중요하다. 큰 스텝 크기(step size)는 개방된 영역에서 진행 속도를 높일 수 있지만 복잡한 장애물 주변에서는 효율성이 낮아질 수 있으며, 작은 스텝은 기하학적 해상도를 향상시키는 대신 추가적인 노드와 검증 연산을 요구한다.

양방향 탐색(bidirectional search)의 주요 장점은 두 개의 신속 탐색 트리가 각각의 탐색 구조가 이동해야 하는 실질적인 거리를 줄일 수 있다는 것이다. 단방향 RRT(unidirectional RRT)는 시작점에서 목표 영역까지 이어지는 일련의 확장을 발견해야 한다. 반면 RRT-Connect는 양쪽 방향에서 중간 자유 공간 영역을 발견할 수 있다. 두 확장 전선(expanding frontier)이 충분히 가까워지면 적극적인 CONNECT 연산을 통해 남아 있는 간격을 연결하고 계획을 종료할 수 있다.

이러한 동작은 로봇 매니퓰레이터 계획(robotic manipulator planning)과 같은 고차원 구성 공간(high-dimensional configuration space)에서 특히 효과적이다. 로봇 팔(robot arm)은 자기 충돌 제약(self-collision constraint)과 작업 공간 장애물(workspace obstacle)을 포함하는 복잡한 관절 공간(joint space)을 통과해야 할 수 있다. 초기 관절 구성과 목표 관절 구성 양쪽에서 트리를 성장시키면 계획기는 양방향에서 실행 가능한 접근 경로를 탐색할 수 있다. 이러한 특성으로 인해 RRT-Connect는 충돌 없는 매니퓰레이터 운동을 신속하게 얻기 위한 실용적인 방법으로 널리 사용된다.

좁은 통로(narrow passage)는 무작위 샘플링이 제한된 영역 내부에서 유용한 구성을 생성할 가능성이 낮기 때문에 여전히 어려운 문제이다. 양방향 성장은 하나의 트리가 어려운 통로의 한쪽에서 접근하고 다른 트리가 반대쪽에서 접근할 가능성을 높일 수 있지만 샘플링 문제 자체를 제거하지는 않는다. 따라서 제약된 통로가 계획 환경을 지배하는 경우 장애물 인식 샘플링(obstacle-aware sampling), 적응형 확장 파라미터(adaptive extension parameter) 또는 특수 샘플링 전략(specialized sampling strategy)을 RRT-Connect와 결합할 수 있다.

두 트리가 만나면 경로 복원(path reconstruction) 과정에서 서로 반대 방향으로 구성된 트리 구조를 고려해야 한다. 시작 트리(start tree)는 시작점으로 돌아가는 부모 관계(parent relationship)를 포함하고 있으며, 목표 트리(goal tree)는 목표점으로 돌아가는 부모 관계를 포함한다. 계획기는 두 가지 가지(branch)를 각각 추출하고 필요한 시퀀스를 역순으로 변환한 다음 연결 구성(connection configuration)에서 결합한다. 이렇게 생성된 경로는 초기 구성에서 목적지까지 이어지는 연속적인 충돌 없는 구성 시퀀스(collision-free configuration sequence)를 형성한다.

기본 RRT와 마찬가지로 RRT-Connect는 최적성(optimality)보다는 빠른 실행 가능성(rapid feasibility)을 주요 목표로 한다. 처음 발견된 해에는 불필요한 회전, 긴 우회 경로 또는 불규칙한 구성 전환이 포함될 수 있다. 따라서 실제 구현에서는 트리 연결 이후 단축(shortcutting)과 경로 평활화(path smoothing)를 자주 적용한다. 경로상의 임의의 구성 쌍 사이에 직접적인 충돌 없는 연결이 가능한지를 검사하여 중간 구간을 제거함으로써 궤적 생성(trajectory generation) 이전에 더 짧고 단순한 경로를 만들 수 있다.

RRT-Connect는 개념적으로 RRT\* 및 Informed RRT\*와 차이가 있다. RRT\*는 부모 선택(parent selection)과 재배선(rewiring)에 추가적인 계산을 사용하여 해의 품질을 점근적으로 개선하는 반면, Informed RRT\*는 초기 해가 발견된 이후 비용을 감소시킬 가능성이 있는 영역으로 샘플링을 집중한다. RRT-Connect는 이와 달리 실행 가능한 연결을 빠르게 발견하는 데 중점을 둔다. 따라서 응용 시스템이 응답 시간, 경로 품질, 반복적인 최적화 또는 계산 예산 가운데 무엇을 우선하는지에 따라 적절한 계획기를 선택해야 한다.

완전한 운동 계획 아키텍처(motion-planning architecture)에서 양방향 RRT-Connect는 충돌 없는 기하학적 경로(collision-free geometric path)를 제공하며, 이후 평활화(smoothing), 궤적 생성(trajectory generation), 동적 실행 가능성(dynamic feasibility) 처리 단계로 전달할 수 있다. 샘플링 기반 계획(sampling-based planning)의 전체 구조에서는 PRM, RRT, RRT\*, Informed RRT\* 이후에 RRT-Connect가 위치하고, 이후 OMPL 활용, 매니퓰레이터 계획(manipulator planning), 키노다이내믹 RRT(kinodynamic RRT), 실시간 계획(real-time planning), MoveIt2 통합으로 이어진다. RRT-Connect의 핵심적인 기여는 양쪽 끝에서 수행하는 탐색(two-ended exploration)과 적극적인 연결 시도(aggressive connection attempt)를 결합하여 복잡한 단일 질의 로봇 운동 계획(single-query robot motion planning) 문제를 신속하게 해결하는 실용적인 메커니즘을 제공하는 데 있다.

## 03.06. OMPL Open Motion Planning Library Usage [w/Code]

![](images/image6.png){width="7.268055555555556in" height="7.268055555555556in"}

오픈 모션 플래닝 라이브러리(Open Motion Planning Library, OMPL)는 공통 계획 인터페이스(common planning interface)를 통해 다양한 샘플링 기반 운동 계획(sampling-based motion planning) 알고리즘을 제공하는 오픈소스 소프트웨어 프레임워크(open-source software framework)이다. 로봇 개발자가 PRM, RRT, RRT\*, RRT-Connect 및 관련 계획기를 각각 독립적으로 구현하는 대신, OMPL은 계획 알고리즘을 로봇별 상태 표현(robot-specific state representation)과 충돌 검사(collision checking)로부터 분리한다. 이러한 모듈형 아키텍처(modular architecture)를 통해 일관된 소프트웨어 환경에서 여러 계획기를 비교하고 적용할 수 있다.

OMPL은 주로 연속 상태 공간(continuous state space)에서의 기하학적 운동 계획(geometric motion planning)과 제어 기반 운동 계획(control-based motion planning)에 초점을 둔다. 라이브러리 자체가 완전한 로봇 내비게이션이나 조작 시스템을 제공하는 것은 아니다. 대신 응용 프로그램이 상태 공간(state space), 유효 상태(valid state), 시작 및 목표 조건, 계획 제약조건을 정의하고 OMPL이 실행 가능하거나 최적화된 운동을 탐색한다. 충돌 검출(collision detection)은 일반적으로 외부 로봇 프레임워크나 응용 프로그램에서 제공하므로 계획 알고리즘은 특정 로봇 형상이나 인지 시스템에 독립적으로 유지될 수 있다.

OMPL의 기본 개념 중 하나는 계획기가 탐색하는 로봇 구성을 수학적으로 표현하는 상태 공간(state space)이다. 단순한 응용에서는 병진 변수를 표현하기 위해 실수 벡터 공간(real-vector space)을 사용할 수 있으며, 이동 로봇은 일반적으로 위치와 방향을 포함하는 SE(2) 또는 SE(3) 표현을 필요로 한다. 매니퓰레이터(manipulator)는 다차원 관절 공간(multidimensional joint space)을 사용할 수 있고, 복합 상태 공간(compound state space)은 여러 표현을 결합할 수 있다. 샘플링, 거리 계산, 보간, 경계 및 이웃 관계가 모두 이러한 표현에 의존하므로 올바른 상태 공간 모델링이 중요하다.

상태 유효성 검사기(state validity checker)는 샘플링된 상태가 허용 가능한지를 판단한다. 응용 프로그램은 장애물과의 충돌, 자기 충돌(self-collision), 관절 한계(joint limit), 금지 영역 또는 기타 실행 가능성 조건을 평가하는 콜백(callback)이나 이에 상응하는 인터페이스를 제공한다. OMPL은 샘플링과 간선 검증(edge validation) 과정에서 이 함수를 반복적으로 호출한다. 유효성 검사는 매우 빈번하게 수행될 수 있으므로 특히 기하학적으로 복잡한 로봇에서는 계산 효율성이 전체 계획 성능에 큰 영향을 미친다.

계획은 일반적으로 상태 공간과 운동 평가에 필요한 정보를 결합하는 공간 정보 구조(space information structure)를 정의하는 것에서 시작한다. 상태 경계(state bounds)를 지정하고 유효성 검사를 등록하며 운동 검증 파라미터(motion validation parameter)를 설정한다. 이후 응용 프로그램은 시작 상태와 하나 이상의 목표 조건을 포함하는 계획 문제(planning problem)를 정의한다. 이러한 분리 구조를 통해 계획기, 시작 및 목표 구성, 최적화 목적(optimization objective), 환경에 의존하는 유효성 로직을 변경하면서 동일한 계획 인프라를 재사용할 수 있다.

OMPL은 서로 다른 탐색 특성을 갖는 다양한 계획기(planner)를 제공한다. PRM과 관련 로드맵 방식은 재사용 가능한 연결 구조가 필요한 경우에 유용하며, RRT와 RRT-Connect는 빠른 탐색과 실행 가능한 해의 발견에 중점을 둔다. RRT\* 및 기타 점근적 최적 계획기(asymptotically optimal planner)는 계산이 계속됨에 따라 경로 품질을 점진적으로 개선한다. 따라서 계획기 선택은 문제의 차원, 장애물 형상, 질의 패턴(query pattern), 사용 가능한 계획 시간 및 실행 가능성과 최적화 중 무엇이 주요 목표인지에 따라 결정해야 한다.

간편 설정 인터페이스(SimpleSetup interface)는 비교적 적은 응용 코드로 다양한 기하학적 계획 문제를 구성할 수 있는 편리한 방법을 제공한다. 개발자는 상태 공간을 정의하고 유효성 검사기를 등록하며 시작 및 목표 상태를 지정하고 필요하면 계획기를 선택한 후 계획 시간 제한과 함께 해결 연산(solve operation)을 호출한다. SimpleSetup은 여러 하위 수준 OMPL 객체를 내부적으로 관리하므로 학습, 프로토타이핑(prototyping), 벤치마킹(benchmarking), 그리고 모든 계획 구성요소를 세밀하게 사용자 정의할 필요가 없는 응용에 유용하다.

계획기 설정(planner configuration)은 실제 성능에 큰 영향을 미친다. RRT 계열 계획기는 확장 범위(extension range)와 목표 편향(goal bias) 관련 파라미터를 제공할 수 있으며, 로드맵 또는 최적 계획기는 이웃 관계와 샘플링 관련 설정을 사용할 수 있다. OMPL은 많은 파라미터에 합리적인 기본값(default)을 제공하지만 이러한 값이 모든 로봇과 환경에 최적일 수는 없다. 따라서 작업 공간 규모, 구성 공간 차원, 장애물 밀도, 좁은 통로, 충돌 검사 비용 및 계획 지연 시간과 경로 품질 사이의 요구되는 균형을 고려하여 설정해야 한다.

계획기가 성공을 보고하면 생성된 해는 일반적으로 기하학적 경로(geometric path)를 형성하는 상태 시퀀스(state sequence)로 표현된다. 샘플링 기반 계획을 통해 생성된 원시 경로(raw path)는 불필요한 중간 구성이나 불규칙한 구간을 포함할 수 있다. OMPL은 단축(shortcutting), 정점 감소(vertex reduction) 및 관련 개선을 시도할 수 있는 경로 단순화(path simplification) 연산을 지원한다. 이후 하위 처리 과정에서 더 촘촘한 구성이 필요하다면 보간(interpolation)을 통해 웨이포인트 밀도(waypoint density)를 높일 수 있다. 이러한 연산은 경로의 활용성을 향상시키지만 완전한 동적 궤적 최적화(dynamic trajectory optimization)와 동일한 것은 아니다.

최적화 목적(optimization objective)을 통해 계획 품질을 명시적으로 표현할 수 있다. 일반적인 목적은 기하학적 경로 길이(geometric path length)를 최소화하는 것이지만 계획 공식이 지원하는 경우 응용 프로그램은 다른 비용 기준을 정의하거나 여러 기준을 결합할 수 있다. 최적 계획기(optimal planner)는 이러한 목적 함수를 이용하여 후보 해를 비교하고 비용을 점진적으로 감소시킨다. 최단 기하학적 거리가 반드시 실제 로봇의 실행 시간, 에너지 소비, 위험도 또는 동작 난이도를 최소화하는 것은 아니므로 목적 함수의 선택은 실제 운용 요구사항과 일치해야 한다.

벤치마킹(benchmarking)은 OMPL의 또 다른 중요한 활용 분야이다. 공통 인터페이스를 통해 동일한 계획 문제에서 여러 계획기의 성능을 평가할 수 있기 때문이다. 개발자는 반복적인 시험을 통해 성공률(success rate), 해 탐색 시간(solution time), 경로 길이(path length) 및 기타 성능 지표를 비교할 수 있다. 샘플링 기반 알고리즘은 확률적(stochastic)이므로 하나의 성공적인 실행 결과에 의존하지 않고 여러 무작위 시드(random seed)와 통계적으로 의미 있는 반복 시험을 사용해야 한다. 이를 통해 성능 차이가 체계적인 결과인지 단순한 무작위 샘플링의 결과인지 판단할 수 있다.

매니퓰레이터 운동 계획(manipulator motion planning)에서 OMPL은 상위 수준 로봇 소프트웨어 아래에서 샘플링 기반 계획 엔진(sampling-based planning engine)으로 자주 사용된다. 응용 계층(application layer)은 로봇 기구학(robot kinematics), 관절 제약조건, 충돌 형상(collision geometry), 계획 장면 정보(planning-scene information)를 제공하고 OMPL은 이에 대응하는 구성 공간을 탐색한다. 이러한 분리는 전체 계획 인프라를 다시 구현하지 않고 동일한 로봇 모델에서 여러 샘플링 전략과 계획기를 평가할 수 있기 때문에 자유도가 높은 매니퓰레이터(high-degree-of-freedom manipulator)에서 특히 유용하다.

OMPL은 단순한 기하학적 보간(geometric interpolation)만으로 충분하지 않은 경우 제어 기반 계획(control-based planning)도 지원할 수 있다. 이러한 문제에서는 상태가 구성 공간의 임의적인 직선 연결이 아니라 제어 입력(control input)과 전파 모델(propagation model)에 따라 변화한다. 이 공식은 미분 제약조건(differential constraint), 조향 제한 또는 동적 거동을 갖는 시스템과 관련된다. 그러나 정확한 차량 또는 로봇 동역학 모델링은 주변 응용 프로그램의 책임이며, 계산 요구량은 순수한 기하학적 계획보다 상당히 증가할 수 있다.

실제 운용 시스템(production system)에서 OMPL은 완전한 자율 시스템(autonomy stack)으로 취급하기보다 위치 추정(localization), 인지(perception), 충돌 모델(collision model), 궤적 생성(trajectory generation), 제어(control), 실행 모니터링(execution monitoring)과 통합해야 한다. 일반적인 작업 흐름에서는 현재 로봇 상태와 작업 목표를 계획 문제로 변환하고 OMPL을 사용하여 충돌 없는 상태 시퀀스를 얻은 다음, 해당 경로를 단순화하거나 최적화하여 하위 궤적 및 제어 구성요소로 전달한다. 환경이 변화하면 경로 재검증(path revalidation)이나 재계획(replanning)이 필요할 수 있다.

전체 운동 계획 구조에서 OMPL은 PRM, RRT, RRT\*, Informed RRT\*, 양방향 RRT-Connect의 이론적 내용을 실제로 적용하고 비교할 수 있는 실용적인 소프트웨어 프레임워크를 제공한다. 또한 이후의 매니퓰레이터 계획(manipulator planning), 키노다이내믹 RRT(kinodynamic RRT), 실시간 샘플링 계획(real-time sampling planning), MoveIt2 통합으로 이어지는 구현 기반을 형성한다. OMPL은 이러한 의미에서 샘플링 기반 계획 이론(sampling-based planning theory)과 재사용 가능한 로봇 소프트웨어 구현(reusable robotics software implementation)을 연결하는 역할을 수행한다.

## 03.07. Sampling Based Planning for Manipulator Arms [w/Code]

![](images/image7.png){width="7.268055555555556in" height="7.268055555555556in"}

샘플링 기반 계획(sampling-based planning)은 매니퓰레이터 암(manipulator arm)의 운동을 3차원 작업 공간(workspace)에서 직접 계획하는 것이 아니라 고차원 구성 공간(high-dimensional configuration space)에서 계획해야 하기 때문에 특히 중요하다. n개의 독립적으로 제어되는 관절을 가진 매니퓰레이터는 일반적으로 n차원 구성 벡터(configuration vector)를 가지며, 각 점은 완전한 로봇 팔 자세를 나타낸다. 따라서 관절 수가 적당히 증가하는 것만으로도 전체 공간을 전수적으로 이산화(discretization)하기 어려워지므로 무작위 샘플링(randomized sampling)에 기반한 계획기가 필요하다.

매니퓰레이터의 구성 공간(configuration space)은 일반적으로 관절 변수(joint variable)를 통해 정의된다. 회전 관절(revolute joint)은 각도 좌표를 제공하고 직동 관절(prismatic joint)은 병진 좌표를 제공한다. 관절 한계(joint limit)는 이 공간에서 허용 가능한 경계를 정의하며 추가적인 제약조건이 유효한 구성을 더욱 제한할 수 있다. 샘플링 기반 계획기는 이러한 경계 내에서 후보 관절 구성을 생성하고 로봇 기구학(robot kinematics)과 충돌 검사(collision checking)를 이용하여 각 구성이 실행 가능한 운동에 포함될 수 있는지를 판단한다.

매니퓰레이터의 충돌 검사(collision checking)는 단순한 이동 로봇 풋프린트(mobile robot footprint)를 검사하는 것보다 복잡하다. 샘플링된 각각의 구성은 여러 링크(link)의 위치와 방향을 변화시키므로 순기구학(forward kinematics)을 통해 해당 로봇 형상을 결정해야 한다. 생성된 구성은 로봇 링크와 환경 객체 사이의 충돌뿐만 아니라 매니퓰레이터의 서로 다른 부분 사이에서 발생하는 자기 충돌(self-collision)에 대해서도 검사해야 한다. 이러한 반복적인 기하학적 검사는 매니퓰레이터 계획에서 계산 비용의 대부분을 차지하는 경우가 많다.

자기 충돌 제약조건(self-collision constraint)은 수학적으로 유효한 관절값이라도 링크들이 물리적으로 불가능하게 서로 교차할 수 있기 때문에 관절형 로봇(articulated robot)에서 특히 중요하다. 로봇 모델에 따라 인접한 링크는 불필요한 충돌 검사에서 제외할 수 있지만 비인접 링크는 세밀하게 검사해야 하는 경우가 많다. 효율적인 충돌 행렬(collision matrix), 경계 볼륨(bounding volume), 계층적 기하 표현(hierarchical geometry representation), 조기 제거 기법(early rejection technique)을 사용하면 비용이 높은 기하학적 비교 횟수를 줄일 수 있다.

샘플링은 관절 공간(joint space)의 구조도 고려해야 한다. 각 관절의 한계 범위에서 균일하게 값을 선택하는 방법은 간단한 기준 방법을 제공하지만, 장애물과 작업 제약조건이 존재하는 경우 유용한 구성은 전체 공간 중 매우 작은 부분만 차지할 수 있다. 목표 편향 샘플링(goal-biased sampling), 장애물 인식 샘플링(obstacle-aware sampling), 제약조건 샘플링(constraint sampling) 또는 기타 특수 전략을 사용하면 보다 관련성이 높은 영역에 탐색을 집중할 수 있다. 따라서 샘플링 전략은 실행 가능한 매니퓰레이터 운동을 얼마나 빠르게 발견할 수 있는지에 큰 영향을 미친다.

최근접 이웃 탐색(nearest-neighbor search)에 사용되는 거리 척도(distance metric)는 관절 구성을 적절하게 비교할 수 있어야 한다. 단순한 거리 척도는 관절각 차이의 가중 노름(weighted norm)을 계산할 수 있지만 관절마다 범위, 물리적 영향 및 회전 위상(rotational topology)이 서로 다를 수 있다. 필요한 경우 각도의 순환 연결성(angular wraparound)을 올바르게 처리해야 하며, 특정 관절이 단순히 수치적 스케일 때문에 거리 척도를 지배하지 않도록 가중치를 적용할 수 있다. 이러한 거리 척도는 트리 확장과 로드맵 연결성에 직접적인 영향을 미친다.

지역 계획(local planning)은 샘플링된 두 매니퓰레이터 구성을 연결할 수 있는지를 결정한다. 기하학적 계획(geometric planning)에서는 일반적으로 두 관절 벡터 사이를 보간(interpolation)하고 중간 구성을 충돌 여부에 대해 검사한다. 이동하는 링크가 검사된 상태 사이에서 장애물을 통과하는 현상을 방지하려면 보간 해상도(interpolation resolution)가 충분히 세밀해야 한다. 적응형 운동 검증(adaptive motion validation)을 사용하면 단순한 영역에서는 불필요한 검사를 줄이면서 충돌 경계 주변에서는 충분한 해상도를 유지하여 효율성을 향상시킬 수 있다.

PRM은 비교적 안정적인 작업 공간에서 많은 계획 질의(planning query)가 반복되는 매니퓰레이터 환경에 유용하다. 충돌 없는 관절 구성으로 로드맵(roadmap)을 구성한 후 서로 다른 시작 자세와 목표 자세에 반복적으로 사용할 수 있다. 이러한 방식은 로봇 형상과 주요 장애물이 고정된 구조화된 제조 셀(structured manufacturing cell)에서 특히 적합하다. 초기 로드맵 구성에는 많은 계산 비용이 필요할 수 있지만 이후 질의에서는 이미 발견된 구성 공간의 연결성을 활용할 수 있다.

RRT와 RRT-Connect는 완전한 재사용 로드맵을 구축하지 않고 고차원 관절 공간을 탐색하기 때문에 단일 질의 매니퓰레이터 계획(single-query manipulator planning)에 일반적으로 적합하다. 특히 RRT-Connect는 시작 구성과 목표 구성에서 성장한 두 트리가 서로 적극적으로 연결을 시도하기 때문에 실용적이다. 많은 조작 작업(manipulation task)에서 이러한 양방향 동작(bidirectional behavior)은 대화형 또는 반복적으로 변경되는 계획 요청에 충분히 빠르게 충돌 없는 관절 공간 경로를 제공할 수 있다.

RRT\* 및 관련 최적 계획기(optimal planner)는 최초의 실행 가능한 경로를 최대한 빠르게 얻는 것보다 해의 품질(solution quality)이 더 중요한 경우 사용할 수 있다. 부모 선택(parent selection)과 재배선(rewiring)을 통해 구성 공간 경로의 비용을 점진적으로 개선한다. 그러나 추가적인 이웃 탐색과 충돌 검사는 매니퓰레이터에서 높은 계산 비용을 요구할 수 있다. 따라서 계획 시스템은 특히 환경 조건이 변화할 가능성이 있는 경우 최적화 시간과 실제 실행 요구사항 사이에서 적절한 균형을 유지해야 한다.

매니퓰레이터 계획(manipulator planning)은 충돌 회피 이외에도 다양한 제약조건을 포함하는 경우가 많다. 말단 장치(end effector)는 지정된 방향을 유지하거나 특정 표면 위에 머물고, 직교 좌표계 방향(Cartesian direction)을 따라 이동하거나 파지 안정성(grasp stability)을 유지하며 특정 작업 공간 영역을 회피해야 할 수 있다. 이러한 요구사항은 전체 구성 공간의 제약된 부분집합(constrained subset)을 정의한다. 단순한 무작위 샘플링으로는 이러한 제약을 만족하는 상태를 거의 생성하지 못할 수 있으므로 투영 방법(projection method), 제약 샘플링(constrained sampling), 역기구학(inverse kinematics) 또는 특수 계획기가 필요할 수 있다.

역기구학(inverse kinematics)은 조작 목표가 명시적인 관절 구성이 아니라 말단 장치 자세(end-effector pose)로 지정되는 경우 특히 중요하다. 원하는 직교 좌표계 자세(Cartesian pose)는 여러 개의 유효한 관절 해(joint solution)에 대응할 수 있으며, 각각은 구성 공간에서 서로 다른 계획 목표를 나타낸다. 하나의 역기구학 해만 선택하면 불필요하게 계획 문제를 어렵게 만들거나 해결 불가능하게 만들 수 있다. 따라서 계획 프레임워크는 여러 목표 구성을 평가하고 실행 가능한 연결성을 제공하는 유효한 해 가운데 하나에 계획기가 도달하도록 할 수 있다.

중복성(redundancy)은 기회와 어려움을 동시에 제공한다. 작업에 필요한 것보다 더 많은 자유도(degree of freedom)를 가진 매니퓰레이터는 동일한 말단 장치 자세를 여러 관절 구성으로 달성할 수 있다. 이러한 중복성을 통해 계획기는 장애물을 우회하고 관절 한계를 피하며 조작성(manipulability)을 향상시킬 수 있지만 동시에 구성 공간의 크기도 증가한다. 샘플링 기반 방법은 가능한 모든 해를 명시적으로 열거하지 않고도 여러 자세 계열(posture family)을 탐색할 수 있기 때문에 이러한 대안을 활용하는 데 적합하다.

샘플링 기반 계획으로 생성된 원시 경로(raw path)는 일반적으로 직접 실행 가능한 궤적이 아니라 충돌 없는 관절 구성의 시퀀스(sequence)이다. 여기에는 불필요한 웨이포인트(waypoint), 급격한 관절 방향 변화, 비효율적인 우회 경로가 포함될 수 있다. 경로 단순화(path simplification)와 단축(shortcutting)을 통해 불필요한 구성을 제거한 후 시간 파라미터화(time parameterization)를 수행하여 관절 속도 및 가속도 한계를 만족하는 속도와 시간을 할당한다. 추가적인 궤적 최적화(trajectory optimization)를 통해 평활성, 여유 거리, 에너지 사용량 또는 실행 시간을 더욱 개선할 수 있다.

계획 과정에서는 조작 장면(manipulation scene)의 변화도 고려해야 한다. 객체가 파지되거나 해제되고 이동하거나 작업 공간에 새롭게 추가되면 충돌 모델(collision model)이 변경되고 이에 따라 유효한 구성 공간도 달라진다. 이러한 변화 이전에 계획된 경로는 더 이상 안전하지 않을 수 있다. 실제 시스템은 최신 계획 장면(planning scene)을 유지하고 관련 운동을 다시 검증하며 필요할 경우 재계획(replanning)을 수행한다. 파지된 객체(attached object) 역시 이후의 운동 과정에서는 로봇 충돌 형상의 일부로 포함해야 한다.

샘플링 기반 계획(sampling-based planning)의 전체 구조에서 매니퓰레이터 계획은 OMPL 활용 이후에 위치하며, 앞에서 다룬 PRM, RRT, RRT\*, Informed RRT\*, RRT-Connect의 개념을 관절형 로봇 팔(articulated robot arm)에 적용한다. 이후에는 키노다이내믹 RRT(kinodynamic RRT), 실시간 샘플링 기반 계획(real-time sampling-based planning), MoveIt2 통합으로 이어진다. 매니퓰레이터 샘플링 기반 계획의 핵심 과제는 고차원 관절 공간 탐색(high-dimensional joint-space exploration)을 충돌이 없고 제약조건을 만족하며 궁극적으로 실행 가능한 로봇 팔 운동으로 변환하는 것이다.

## 03.08. Kinodynamic RRT for Differential Drive AMR [w/Code]

![](images/image8.png){width="7.268055555555556in" height="7.268055555555556in"}

키노다이내믹 RRT(Kinodynamic RRT)는 로봇의 운동 제약조건(motion constraint)과 동역학(dynamics)을 트리 확장(tree expansion)에 직접 포함하여 샘플링 기반 계획(sampling-based planning)을 순수한 기하학적 충돌 회피 경로 이상의 영역으로 확장한다. 차동 구동 자율이동로봇(Differential-Drive Autonomous Mobile Robot, AMR)의 경우 구성 사이를 임의의 직선으로 보간하면 플랫폼이 물리적으로 실행할 수 없는 운동이 생성될 수 있다. 키노다이내믹 계획(kinodynamic planning)은 대신 후보 제어 입력(control input)을 운동 모델(motion model)을 통해 전파하여 로봇의 비홀로노믹 제약(nonholonomic constraint)과 동적 제약(dynamic constraint)을 만족하는 도달 가능한 상태(reachable state)를 생성한다.

차동 구동 AMR은 일반적으로 평면 위치와 방향을 포함하는 x = [x, y, θ] 형태의 상태(state)로 표현하며, 가속도와 동적 한계를 명시적으로 모델링해야 하는 경우 선속도(linear velocity)와 각속도(angular velocity) 같은 변수를 추가할 수 있다. 제어 입력은 명령 선속도 v와 각속도 ω로 표현하거나, 이와 동등하게 좌우 바퀴 속도(left and right wheel velocity)로 표현할 수 있다. 선택한 표현 방식은 계획기가 이동 플랫폼의 실제 물리적 거동을 얼마나 정확하게 반영할 수 있는지를 결정한다.

기본 운동학 모델(kinematic model)은 x_dot = v cos θ, y_dot = v sin θ, θ_dot = ω를 통해 상태 변화와 명령 운동 사이의 관계를 표현한다. 이 방정식은 차동 구동 운동의 핵심적인 비홀로노믹 특성(nonholonomic property), 즉 로봇이 순간적으로 측면 방향으로 이동할 수 없다는 특성을 나타낸다. 이러한 제약을 무시하는 기하학적 계획기(geometric planner)는 충돌은 없지만 실제 AMR이 상당한 후처리 없이 추종할 수 없는 측면 이동이나 급격한 방향 변화를 포함하는 경로를 생성할 수 있다.

키노다이내믹 RRT는 현재 로봇 상태(current robot state)를 탐색 트리(search tree)의 루트(root)로 설정하여 시작한다. 각 반복에서 계획기(planner)는 목표 상태(target state) 또는 목표 영역을 샘플링하고 확장에 적합한 기존 트리 상태를 선택한다. 샘플 방향으로 직선 구간을 생성하는 대신 하나 이상의 후보 제어 입력과 전파 시간(propagation duration)을 선택한다. 각각의 후보는 로봇 모델을 통해 시뮬레이션되며, 이를 통해 동적으로 도달 가능한 후속 상태(successor state)와 이에 대응하는 운동 구간(motion segment)을 생성한다.

따라서 제어 샘플링(control sampling)은 키노다이내믹 탐색의 핵심 요소이다. 후보 동작(action)은 전진 속도, 허용되는 경우 후진 속도, 양 또는 음의 각속도 조합을 포함할 수 있다. 제어 입력은 허용 범위 내에서 무작위로 샘플링하거나 목표 상태 방향으로 진행하도록 유도하는 휴리스틱(heuristic)을 사용하여 선택할 수 있다. 전파 시간 역시 신중하게 결정해야 한다. 짧은 지속 시간은 세밀한 기동성을 제공하는 반면 긴 지속 시간은 빠른 탐색을 가능하게 하지만 거친 운동이나 충돌 가능성이 높은 운동을 생성할 수 있다.

상태 전파(state propagation) 과정에서 계획기는 선택된 시간 동안 운동 모델을 수치 적분(numerical integration)한다. 전파된 궤적을 따라 중간 상태(intermediate state)를 생성하고 지도 경계, 장애물, 풋프린트 제약조건(footprint constraint), 속도 한계 및 기타 실행 가능성 조건을 검사한다. 중간 상태 중 하나라도 유효하지 않으면 전파를 거부하거나 마지막 유효 상태에서 종료할 수 있다. 성공적인 전파는 임의의 기하학적 간선이 아니라 실행 가능한 운동 프리미티브(executable motion primitive)로 연결된 새로운 트리 정점을 생성한다.

충돌 검사(collision checking)는 전파 과정 전체에서 AMR의 완전한 풋프린트(footprint)를 고려해야 한다. 로봇을 하나의 점으로 표현하면 차체가 선반, 벽, 설비 또는 다른 장애물과 충돌하는 궤적을 허용할 수 있다. 원형 풋프린트(circular footprint)는 검사를 단순화하며, 직사각형 또는 다각형 모델(rectangular or polygonal model)은 차량의 실제 형상을 더욱 정확하게 표현한다. 또한 안전 팽창(safety inflation)을 적용하여 위치 추정 오차, 추종 오차, 정지 거리 및 운용상 필요한 여유 공간을 고려할 수 있다.

트리 확장을 유도하는 거리 척도(distance metric)는 위치와 방향을 모두 고려해야 한다. x와 y 좌표 사이의 순수한 유클리드 거리(Euclidean distance)만 사용하면 동일한 위치에 있지만 서로 반대 방향을 향하는 두 상태를 동일하게 취급할 수 있다. 가중 거리 척도(weighted metric)는 병진 거리와 각도 차이를 결합할 수 있으며, 속도를 포함하는 상태 공간에서는 추가적인 속도 항(velocity term)이 필요할 수 있다. 거리 척도의 설계는 어떤 트리 노드가 선택되는지와 계획기가 도달 가능한 상태를 얼마나 효율적으로 탐색하는지에 큰 영향을 미친다.

차동 구동 AMR은 비홀로노믹(nonholonomic)이지만 일반적으로 좌우 바퀴에 반대 방향의 속도를 명령하여 회전할 수 있기 때문에 높은 기동성을 갖는다. 제자리 회전(in-place rotation)을 허용할 것인지는 실제 플랫폼과 운용 제약조건에 따라 결정해야 한다. 계획기는 순수 회전을 허용하거나 최소 병진 운동을 강제하고, 후진 주행을 제한하거나 급격한 각속도 명령에 높은 비용을 부여할 수 있다. 따라서 키노다이내믹 계획은 모든 차동 구동 플랫폼이 동일한 능력을 갖는다고 가정하기보다 실제 주행 특성을 모델링해야 한다.

명령이 순간적으로 변화할 수 없는 경우 가속도 한계(acceleration limit)가 중요해진다. 선속도가 0에서 최대 속도로 즉시 증가하거나 각속도의 방향이 순간적으로 반전된다면 기구학적으로는 가능한 경로라도 물리적으로는 비현실적일 수 있다. 상태에 속도 변수를 추가하고 가속도가 제한된 제어(acceleration-limited control)를 전파하면 더욱 현실적인 표현이 가능하다. 이는 상태 공간의 차원을 증가시키지만 액추에이터 성능, 바퀴 접지력, 적재 하중의 영향 및 요구되는 정지 거동을 고려한 계획을 가능하게 한다.

목표 조건(goal condition)은 일반적으로 하나의 정확한 상태가 아니라 목표 영역(goal region)으로 표현한다. AMR의 위치가 지정된 허용 오차 내에 있고 방향이 원하는 방향에 충분히 가까우면 목적지에 도달한 것으로 판단할 수 있다. 도킹(docking)이나 작업 스테이션 정렬(workstation alignment)은 일반적인 내비게이션보다 훨씬 엄격한 허용 오차를 요구할 수 있다. 현실적인 목표 영역을 정의하면 계획기가 불필요하거나 도달하기 어려운 수학적으로 정확한 단일 상태를 찾는 데 계산 자원을 낭비하는 것을 방지할 수 있다.

키노다이내믹 RRT는 기존 RRT의 탐색 장점을 유지하면서 기하학적 조향(geometric steering)을 순방향 시뮬레이션(forward simulation)으로 대체한다. 이는 임의의 두 상태 사이를 직접 연결하는 정확한 조향 함수(steering function)를 사용할 수 없는 경우 특히 유용하다. 차동 구동 운동은 많은 동적 시스템에 비해 비교적 단순하지만 장애물, 속도 제약조건, 방향 요구사항 및 유한한 제어 지속 시간으로 인해 직접적인 상태 간 연결이 어려울 수 있다. 순방향 전파는 실제 모델이 생성할 수 있는 운동만을 이용하여 탐색 구조를 구축하는 실용적인 방법을 제공한다.

생성된 해(solution)는 초기 상태에서 목표 영역까지 이어지는 상태(state), 제어 입력(control), 전파 시간(propagation duration)의 시퀀스로 구성된다. 순수한 기하학적 RRT 경로와 달리 이러한 시퀀스에는 연속된 상태 사이에서 로봇이 어떻게 이동해야 하는지에 대한 정보가 이미 포함되어 있다. 그러나 불필요한 제어 변화를 제거하고 평활성(smoothness)을 향상시키며 더 엄격한 가속도 한계를 적용하거나 계획된 명령을 AMR의 하위 속도 및 모터 제어기 인터페이스로 변환하기 위해 추가적인 궤적 처리(trajectory processing)가 필요할 수 있다.

실시간 AMR 운용(real-time AMR operation)에서는 계획된 운동을 실행하는 동안 환경이 변화할 수 있기 때문에 추가적인 요구사항이 발생한다. 동적 장애물(dynamic obstacle), 위치 추정 갱신(localization update), 새롭게 감지된 위험 요소 및 주행 가능성(traversability)의 변화는 이전에 생성된 트리 가지를 무효화할 수 있다. 따라서 키노다이내믹 RRT는 전역 계획(global planning)이 광범위한 경로 안내를 제공하고 지역 키노다이내믹 계획(local kinodynamic planning) 또는 제어가 현재 로봇 상태와 주변 제약조건을 지속적으로 고려하는 계층형 아키텍처(hierarchical architecture)에서 동작할 수 있다.

계산 효율성(computational efficiency)은 상태 차원(state dimensionality), 제어 샘플링, 전파 해상도(propagation resolution), 충돌 검사 비용 및 트리 크기에 따라 달라진다. 지나치게 조밀한 제어 샘플링은 각각의 확장 비용을 증가시키고 지나치게 희소한 샘플링은 유용한 기동을 놓칠 수 있다. 긴 전파 구간은 탐색을 가속하지만 지역적인 유연성을 감소시킨다. 실제 구현에서는 이러한 요소의 균형을 조정하며 계획 시간 요구사항을 충족하기 위해 편향 샘플링(biased sampling), 운동 프리미티브(motion primitive), 가지치기(pruning), 병렬 후보 평가(parallel candidate evaluation) 또는 재사용 가능한 탐색 정보를 활용할 수 있다.

샘플링 기반 계획(sampling-based planning)의 전체 구조에서 키노다이내믹 RRT는 매니퓰레이터 중심의 샘플링 방법 이후에 위치하며, 앞에서 설명한 RRT 개념을 기하학적 구성 공간 탐색(geometric configuration-space exploration)에서 동역학적으로 실행 가능한 상태 공간 탐색(dynamically feasible state-space exploration)으로 확장한다. 이후 실시간 샘플링 기반 계획(real-time sampling-based planning)과 로봇 계획 프레임워크(robot planning framework) 통합으로 자연스럽게 이어진다. 차동 구동 AMR에서 키노다이내믹 RRT의 핵심적인 기여는 충돌 없는 탐색이 플랫폼의 실제 물리적 운동 방식까지 만족하도록 하여 샘플링된 가지를 단순한 기하학적 연결이 아니라 실제 로봇이 도달 가능한 운동으로 변환하는 데 있다.

## 03.09. Real Time Sampling Planning with Time Constraint

![](images/image9.png){width="7.268055555555556in" height="7.268055555555556in"}

실시간 샘플링 기반 계획(real-time sampling-based planning)은 로봇이 가능한 최상의 경로를 무한정 탐색하는 대신 엄격한 계산 마감시간(computational deadline) 내에서 사용할 수 있는 해를 생성해야 하는 운동 계획(motion planning) 문제를 다룬다. 실제 자율 시스템에서 계획 지연시간(planning latency)은 반응성과 안전성에 직접적인 영향을 미친다. 따라서 계획기(planner)는 허용된 시간 예산(time budget) 내에 유용한 운동 정보를 제공하면서 탐색, 해의 품질, 충돌 검사(collision checking), 최적화 사이의 균형을 유지해야 한다.

시간 제약(time constraint)은 샘플링 기반 계획의 목적 자체를 변화시킨다. 전통적인 알고리즘은 확률적 완전성(probabilistic completeness)이나 점근적 최적성(asymptotic optimality)을 기준으로 분석되는 경우가 많지만, 이러한 특성은 계산 시간이 사실상 무한대로 증가할 때의 동작을 설명한다. 실제 로봇은 유한한 제어 주기(control cycle)와 마감시간 내에서 동작한다. 따라서 핵심 질문은 전역 최적해(global optimum)가 아니더라도 충분히 안전하고 실행 가능한 경로를 할당된 계획 시간이 종료되기 전에 일관되게 생성할 수 있는가가 된다.

계획 시간 예산(planning budget)은 반응형 지역 운동(reactive local motion)의 경우 수 밀리초 수준에서 복잡한 전역 계획(global planning)이나 매니퓰레이터 계획(manipulator planning)의 경우 수백 밀리초 또는 수 초까지 다양할 수 있다. 계획기는 이러한 예산을 엄격한 마감시간(hard deadline) 또는 유연한 목표 시간(soft target)으로 처리할 수 있다. 하드 실시간(hard real-time)은 정의된 마감시간 이전의 완료를 요구하지만, 소프트 실시간 계획(soft real-time planning)은 간헐적인 초과를 허용하면서 예측 가능한 지연시간을 목표로 한다. 대부분의 샘플링 기반 로봇 계획기는 제한된 계산 시간을 제공하지만 엄격한 하드 실시간 실행을 수학적으로 보장하지는 않는다.

RRT 계열 알고리즘(RRT-family algorithm)은 탐색되지 않은 영역으로 빠르게 확장할 수 있고 자유 공간(free space)의 포괄적인 표현을 구축하기 전에 초기 실행 가능 해(initial feasible solution)를 발견하는 경우가 많기 때문에 시간 제약 환경에서 유용하다. 특히 RRT-Connect는 최적성보다 빠른 실행 가능성(fast feasibility)이 중요한 경우 양방향 트리가 적극적으로 연결을 시도하므로 효과적이다. 반면 RRT\*는 더 우수한 최적화 특성을 제공하지만 추가적인 이웃 탐색, 비용 평가, 충돌 검사 및 재배선(rewiring)이 중요한 계산 시간을 소비할 수 있다.

애니타임 계획(anytime planning)은 가변적인 계산 예산을 처리하기 위한 효과적인 전략을 제공한다. 계획기는 먼저 실행 가능한 해를 최대한 빠르게 확보한 다음 남아 있는 시간 동안 해당 해를 개선한다. 마감시간에 도달하면 그 시점까지 발견된 최상의 유효 경로(best valid path)를 즉시 반환한다. RRT\*와 Informed RRT\*는 초기 경로를 발견한 이후에도 해의 비용을 지속적으로 감소시킬 수 있으므로 이러한 방식에 자연스럽게 적합하며, 사용 가능한 계산 시간에 따라 경로 품질을 향상시킬 수 있다.

실제 계획기는 남아 있는 시간 예산(remaining time budget)을 명시적으로 관리해야 한다. 샘플링 반복, 충돌 검증 연산 또는 최적화 단계 사이에서 시간 검사를 수행할 수 있다. 안전하게 완료하기에 충분한 시간이 남아 있지 않다면 비용이 높은 절차를 새롭게 시작하지 않아야 한다. 계획 종료 조건(planning termination condition)은 경과 시간, 최대 반복 횟수, 허용 가능한 해 비용(acceptable solution cost), 외부 중단 신호(external interruption signal)를 결합할 수 있다. 이러한 메커니즘은 통제되지 않은 계산이 후속 궤적 생성과 제어를 지연시키는 것을 방지한다.

충돌 검사(collision checking)는 실시간 샘플링 기반 계획에서 계산 비용의 대부분을 차지하는 경우가 많다. 각각의 샘플 상태, 후보 간선(candidate edge), 전파된 운동(propagated motion), 재배선 연결은 기하학적 유효성 검증을 요구할 수 있다. 효율적인 광역 충돌 검출(broad-phase collision detection), 단순화된 로봇 형상, 공간 인덱싱(spatial indexing), 유효성 결과 캐싱(cached validity result), 적응형 검사 해상도(adaptive checking resolution)를 통해 이러한 부담을 줄일 수 있다. 그러나 계산 가속을 위해 계획 시스템이 요구하는 충돌 안전성 가정을 유지하는 수준 이하로 검증 수준을 낮춰서는 안 된다.

샘플링 전략(sampling strategy)은 제한된 계산 시간을 얼마나 효율적으로 사용하는지에 큰 영향을 미친다. 균일 무작위 샘플링(uniform random sampling)은 넓은 탐색 범위를 제공하지만 현재 작업과 관련이 없는 영역에서 샘플을 낭비할 수 있다. 목표 편향(goal biasing)은 목적지 방향의 탐색을 증가시키고, 정보 기반 샘플링(informed sampling)은 기존 해를 개선할 수 있는 상태로 최적화 영역을 제한할 수 있다. 장애물 인식 샘플링(obstacle-aware sampling)과 경험 기반 샘플링(experience-based sampling)은 탐색을 유용한 영역에 더욱 집중할 수 있다. 엄격한 마감시간에서는 불필요한 샘플을 하나라도 줄이는 것이 사용 가능한 해를 얻을 가능성을 높일 수 있다.

이전 계획 정보(previous planning information)를 재사용하는 것도 지연시간을 줄이는 데 도움이 된다. 반복적인 내비게이션 주기 동안 로봇은 이전 문제와 약간만 다른 계획 문제를 해결하는 경우가 많다. 따라서 이전 경로, 로드맵(roadmap), 탐색 트리(search tree) 또는 샘플 상태 집합에서 여전히 유효한 부분을 재계획(replanning)의 초기 정보로 활용할 수 있다. 웜 스타트 전략(warm-start strategy)은 필요한 탐색량을 줄일 수 있지만 장애물, 위치 추정 결과, 로봇 제약조건 또는 환경 조건이 변경되었다면 재사용되는 정보를 반드시 다시 검증해야 한다.

증분 재계획(incremental replanning)은 동적 환경(dynamic environment)에서 운용되는 자율이동로봇(Autonomous Mobile Robot, AMR)에 특히 유용하다. 새로운 센서 정보가 입력될 때마다 전체 해를 폐기하는 대신 기존 경로에서 유효한 부분을 유지하고 영향을 받은 영역에 계산을 집중할 수 있다. 샘플링 기반 계획기는 느린 전역 계획기가 경로 수준의 안내를 제공하고 빠른 지역 계획기(local planner)가 현재 장애물과 로봇 상태 정보를 사용하여 단기 운동(short-horizon motion)을 지속적으로 생성하는 계층형 아키텍처(hierarchical architecture)에서 동작할 수 있다.

계획 지평(planning horizon)은 계산량을 제어하기 위한 또 다른 메커니즘이다. 전체 장거리 임무를 높은 해상도로 한 번에 계획하는 대신 제한된 공간적 또는 시간적 지평(spatial or temporal horizon)에 대해서만 계획하고 로봇이 이동함에 따라 해를 반복적으로 갱신할 수 있다. 짧은 지평은 상태 공간 탐색과 충돌 검사 요구량을 줄여 환경 변화에 더 빠르게 대응할 수 있게 한다. 그러나 지역적으로 매력적인 결정이 로봇을 막다른 경로나 동역학적으로 위험한 상황으로 유도하지 않도록 충분한 길이의 지평을 유지해야 한다.

키노다이내믹 계획(kinodynamic planning)은 후보 제어 입력을 운동 모델을 통해 전파해야 하기 때문에 추가적인 실시간 계산 비용을 발생시킨다. 각각의 트리 확장 과정에서 여러 제어 샘플, 적분 단계(integration step), 중간 충돌 검사가 필요할 수 있다. 실시간 구현에서는 제어 집합(control set)을 제한하거나 사전 정의된 운동 프리미티브(motion primitive)를 사용하고, 전파 지평(propagation horizon)을 줄이거나 후보 평가를 병렬화하며 단순화된 예측 모델을 사용할 수 있다. 이러한 방법은 모델링 상세도 및 탐색 다양성과 계산 지연시간 사이의 절충을 형성한다.

병렬 계산(parallel computation)은 많은 후보 샘플, 충돌 검사 또는 제어 전파를 독립적으로 평가할 수 있기 때문에 샘플링 기반 계획의 성능을 크게 향상시킬 수 있다. 멀티코어 CPU(multi-core CPU)와 적절한 가속기(accelerator)는 여러 계획 연산을 동시에 처리할 수 있지만 동기화(synchronization)와 공유 트리 갱신(shared tree update)은 추가적인 오버헤드를 발생시킨다. 따라서 병렬화는 계산 비용이 높은 연산을 과도한 자원 경합(contention)이나 비결정적 타이밍(nondeterministic timing)을 발생시키지 않으면서 충분히 독립적인 작업으로 분리할 수 있을 때 가장 효과적이다.

예측 가능성(predictability)은 평균적인 계획 속도만큼 중요하다. 일반적으로 20 ms 내에 경로를 반환하지만 간헐적으로 500 ms가 필요한 계획기는 50 ms마다 갱신을 요구하는 제어 아키텍처(control architecture)에 적합하지 않을 수 있다. 따라서 성능 평가에서는 평균 실행 시간만이 아니라 지연시간 분포(latency distribution)를 분석해야 한다. 중앙값(median), 백분위수(percentile), 관측된 최악 실행 시간(worst-case observed planning time), 마감시간 이전 성공률(success rate before deadline), 경로 품질 및 충돌 검사 작업량을 함께 평가해야 실시간 계획 성능을 보다 의미 있게 특성화할 수 있다.

마감시간 이전에 실행 가능한 해가 발견되지 않을 수 있으므로 실패 처리(failure handling)를 명시적으로 설계해야 한다. 계산 시간이 종료되었다는 이유만으로 로봇이 유효하지 않거나 부분적으로만 검증된 경로를 실행해서는 안 된다. 응용 시스템에 따라 이전에 검증된 궤적(trajectory)을 유지하거나 속도를 낮추고, 안전하게 정지하거나 통제된 정지 상태에서 계획 시간을 연장하거나 대체 계획기(alternative planner)를 호출할 수 있다. 따라서 계획 실패(planning failure)는 감독(supervision) 및 제어 시스템과 통합되어야 하는 하나의 운용 상태(operational state)이다.

실시간 샘플링 계획은 궁극적으로 인지(perception), 위치 추정(localization), 계획(planning), 궤적 생성(trajectory generation), 제어(control) 사이의 협조를 필요로 한다. 새로운 환경 정보는 상태와 지도의 갱신을 유발하고, 계획기는 주어진 시간 예산 내에서 실행 가능한 운동을 생성하며, 궤적 처리 과정은 그 결과를 실행 가능한 명령으로 변환한다. 이후 실행 모니터링(execution monitoring)은 재계획이 필요한지를 판단한다. 핵심적인 공학적 목표는 단순히 최대 탐색 성능을 얻는 것이 아니라 물리적 로봇 운용이 요구하는 시간 제약 내에서 안전하고 충분한 품질의 운동 결정을 신뢰성 있게 제공하는 것이다.

## 03.10. MoveIt2 OMPL Integration for Arm Planning [w/Code]

![](images/image10.png){width="7.268055555555556in" height="7.268055555555556in"}

MoveIt2는 ROS 2 생태계(ROS 2 ecosystem) 내에서 로봇 모델링(robot modeling), 장면 표현(scene representation), 기구학(kinematics), 충돌 검사(collision checking), 운동 계획(motion planning), 궤적 처리(trajectory processing), 실행(execution)을 통합한다. 매니퓰레이터 암(manipulator arm)의 경우 OMPL은 일반적으로 이 프레임워크의 하위에서 샘플링 기반 계획 알고리즘(sampling-based planning algorithm)을 제공한다. MoveIt2가 로봇별 정보와 계획 요청을 제공하면 OMPL 계획 파이프라인(planning pipeline)은 해당 구성 공간(configuration space)을 탐색하여 충돌 없는 경로를 찾는다. 이러한 분리를 통해 계획 알고리즘을 다양한 로봇 팔과 응용 분야에서 재사용할 수 있다.

통합은 정확한 로봇 모델(robot model)에서 시작된다. URDF는 매니퓰레이터의 링크(link), 관절(joint), 형상(geometry), 물리적 구조를 기술하고, SRDF는 계획 그룹(planning group), 말단 장치(end effector), 사전 정의 상태(predefined state), 수동 관절(passive joint), 충돌 관계(collision relationship)와 같은 의미론적 정보를 제공한다. 계획 그룹은 일반적으로 하나의 로봇 팔이나 다른 제어 가능한 메커니즘에 속하는 관절들을 지정한다. MoveIt2는 이러한 정보를 기구학, 상태 유효성 검사(state validity checking), 충돌 검출(collision detection), OMPL 상태 공간 구성에 사용되는 내부 로봇 모델로 변환한다.

계획 장면(planning scene)은 로봇 팔이 움직여야 하는 환경을 표현한다. 현재 로봇 상태(current robot state)와 충돌 객체(collision object), 부착 객체(attached object), 허용 충돌 정보(allowed-collision information), 그리고 계획과 관련된 기타 기하학적 제약조건을 결합한다. 객체를 파지하면 해당 객체를 로봇 링크에 부착하여 이후의 충돌 검사에서 움직이는 시스템의 일부로 취급할 수 있다. OMPL은 MoveIt2가 제공하는 장면을 기준으로 후보 상태와 운동을 평가하므로 정확한 계획 장면을 유지하는 것이 중요하다.

OMPL 자체가 완전한 물리적 로봇을 독립적으로 이해하는 것은 아니다. MoveIt2의 OMPL 인터페이스는 계획 그룹을 적절한 구성 공간 표현(configuration-space representation)으로 매핑하고 선택된 계획기에 필요한 유효성 정보를 제공한다. 샘플링된 구성은 관절 상태(joint state)에 대응하며, MoveIt2는 이러한 상태가 관절 경계(joint bound), 충돌 요구조건, 적용 가능한 제약조건을 만족하는지 평가한다. 이 아키텍처에서 OMPL은 탐색(search)을 수행하고 MoveIt2는 샘플링된 상태와 연결이 의미 있는지를 결정하는 로봇 공학적 문맥(robotics context)을 제공한다.

계획 요청(planning request)은 목표 관절 구성(target joint configuration), 말단 장치 자세(end-effector pose) 또는 지원되는 다른 목표 및 경로 제약조건(path constraint)을 지정할 수 있다. 직교 좌표계 말단 장치 목표(Cartesian end-effector target)가 주어지면 역기구학(inverse kinematics)을 사용하여 원하는 자세를 만족할 수 있는 관절 구성을 구한다. 매니퓰레이터의 중복성(redundancy)으로 인해 여러 개의 유효한 관절 해가 존재할 수 있다. 따라서 계획 시스템은 미리 결정된 관절값 시퀀스를 요구하는 대신 목표를 만족하는 구성으로 이어지는 충돌 없는 경로를 탐색할 수 있다.

계획기 설정(planner configuration)은 어떤 OMPL 알고리즘을 사용할 것인지와 해당 알고리즘이 어떻게 동작할지를 결정한다. 일반적인 선택에는 빠르게 실행 가능한 계획을 수행하는 RRTConnect와 지속적인 개선이 중요한 경우 사용하는 RRT\*와 같은 점근적 최적 계획기(asymptotically optimal planner)가 포함된다. 계획 시간(planning time), 시도 횟수(number of attempts), 확장 범위(extension range), 목표 편향(goal bias), 계획기별 설정 등의 파라미터가 성능에 영향을 미친다. 적절한 설정은 로봇 자유도(degree of freedom), 작업 공간 형상, 장애물 밀도, 좁은 통로, 충돌 검사 비용 및 요구되는 응답 시간에 따라 달라진다.

RRTConnect는 로봇 팔 운동이 일반적으로 고차원 관절 공간(high-dimensional joint space)의 단일 질의 문제(single-query problem)이기 때문에 매니퓰레이터 계획에서 자주 효과적으로 사용된다. 하나의 트리는 현재 로봇 구성에서 성장하고 다른 트리는 목표 구성에서 성장하며, 두 트리는 충돌 없는 공간을 통해 서로 연결을 시도한다. 이러한 적극적인 양방향 탐색(aggressive bidirectional exploration)은 실행 가능한 경로를 빠르게 생성하는 경우가 많다. 그러나 최초의 해가 짧거나 부드러운 경로라는 보장은 없으므로 실행 전에 후속 경로 처리(path processing) 단계가 중요하다.

충돌 검사(collision checking)는 MoveIt2와 OMPL 사이의 핵심적인 인터페이스를 형성한다. 각각의 샘플 상태(sampled state) 또는 후보 운동(candidate motion)은 자기 충돌(self-collision)과 환경 충돌(environmental collision)에 대한 검증을 요구할 수 있다. MoveIt2는 로봇 충돌 모델(robot collision model)과 계획 장면을 이용하여 이러한 검사를 수행한다. 개별적으로 유효한 두 끝점이 그 사이의 로봇 팔 운동까지 충돌이 없음을 보장하지 않으므로 운동 검증(motion validation)에서는 샘플 상태 사이의 중간 구성도 검사해야 한다. 따라서 충돌 검사 해상도(collision-checking resolution)는 안전성과 계획 계산량 모두에 영향을 미친다.

제약조건(constraint)은 로봇 팔 계획을 제약이 없는 관절 공간 탐색보다 더욱 어렵게 만든다. 작업에 따라 말단 장치가 특정 방향을 유지하거나 지정된 위치 영역 내에 머물러야 하며, 장애물을 회피하면서 다른 경로 제한을 따라야 할 수도 있다. 이러한 요구사항은 구성 공간에서 실행 가능한 영역을 크게 감소시킬 수 있다. MoveIt2는 작업 수준 제약조건(task-level constraint)을 표현하고 관련 유효성 요구사항을 계획 과정에 전달하며, 유효 상태가 좁은 다양체(manifold)에 존재하는 경우 제약 샘플링(constrained sampling)이나 특수 계획 방법이 필요할 수 있다.

OMPL이 해를 발견하면 그 결과는 기본적으로 로봇 구성 공간을 통과하는 기하학적 경로(geometric path)이다. 이 경로는 계획 과정에서 사용된 유효성 조건을 만족하면서 초기 구성과 목표를 연결하는 관절 상태의 집합으로 구성된다. 그러나 그 자체로 완전한 시간 파라미터화 명령 시퀀스(time-parameterized command sequence)를 의미하지는 않는다. 따라서 MoveIt2는 기하학적 해를 단순화하고 조정하여 궤적 생성(trajectory generation)과 최종적인 로봇 제어 시스템 실행에 적합하도록 추가적인 처리를 수행한다.

경로 단순화(path simplification)는 무작위 샘플링으로 생성된 불필요한 웨이포인트(waypoint)와 불규칙한 우회 구간을 제거할 수 있다. 기하학적 처리 이후 시간 파라미터화(time parameterization)는 설정된 관절 한계(joint limit)를 준수하면서 시간, 속도, 가속도 정보를 할당한다. 이 단계는 계획된 관절 공간 경로를 실행 가능한 로봇 궤적(robot trajectory)으로 변환한다. 이러한 구분은 중요하다. OMPL이 주로 로봇 팔이 구성 공간의 어디를 통해 이동할 수 있는지를 결정한다면, 궤적 처리는 해당 운동이 시간에 따라 어떻게 진행되어야 하는지를 결정한다.

실행(execution)은 계획 파이프라인을 실제 또는 시뮬레이션된 매니퓰레이터와 연결한다. 생성된 궤적은 적절한 MoveIt2 실행 인터페이스를 통해 관절 명령을 담당하는 ROS 2 제어기(controller)로 전달된다. 실행 중 실제 로봇 상태는 계획에서 사용한 가정과 일치해야 한다. 로봇 상태에 상당한 편차가 발생하거나 제어기 고장, 새롭게 감지된 장애물 또는 계획 장면 변화가 발생하면 실행을 중지하거나 새로운 계획 요청을 생성해야 할 수 있다.

MoveIt2는 응용 프로그램이 저수준 OMPL 객체를 직접 다루지 않고도 계획 요청을 구성할 수 있는 인터페이스를 제공한다. 상위 수준 소프트웨어는 계획 그룹, 현재 상태, 목표, 제약조건 및 계획 파라미터를 지정한 후 계획(planning) 또는 계획 및 실행(planning-and-execution)을 요청할 수 있다. 이러한 추상화(abstraction)는 작업 로직(task logic)을 샘플링, 최근접 이웃 탐색(nearest-neighbor search), 트리 구성(tree construction), 충돌 검증, 계획 종료와 같은 세부 구현으로부터 분리할 수 있기 때문에 조작 응용(manipulation application)에서 유용하다.

OMPL 알고리즘은 일반적으로 확률적(stochastic)이므로 계획기 성능은 통계적으로 평가해야 한다. 동일한 시작 및 목표 조건에서도 반복 실행마다 서로 다른 경로와 계산 시간이 생성될 수 있다. 따라서 실제 평가에서는 여러 번의 시험을 통해 계획 성공률(planning success rate), 지연시간 분포(latency distribution), 경로 길이(path length), 궤적 지속 시간(trajectory duration), 여유 거리(clearance), 실패 사례를 함께 고려해야 한다. 이후 특정 매니퓰레이터와 환경에 맞게 계획기 파라미터를 조정해야 하며 기본 설정이 모든 환경에 보편적으로 적합하다고 가정해서는 안 된다.

동적인 조작 환경(dynamic manipulation environment)에서는 계획 장면을 인지(perception) 및 작업 실행(task execution)과 동기화해야 한다. 객체가 이동하거나 나타나거나 사라지고 또는 로봇에 부착될 수 있으며, 이에 따라 계획에서 사용하는 충돌 관계가 변경된다. 따라서 이전에 유효했던 궤적이 더 이상 안전하지 않을 수 있다. 신뢰성 있는 통합을 위해서는 장면 갱신(scene update), 상태 모니터링(state monitoring), 궤적 검증(trajectory validation), 재계획 정책(replanning policy)이 필요하며, 이를 통해 실제 환경이 원래 경로를 생성할 때의 가정과 달라졌을 경우 운동 계획 파이프라인이 적절하게 대응할 수 있어야 한다.

샘플링 기반 계획(sampling-based planning)의 전체 구조에서 MoveIt2-OMPL 통합은 PRM과 RRT 이론에서 시작하여 RRT\*, Informed RRT\*, RRT-Connect, OMPL 활용, 매니퓰레이터 계획, 키노다이내믹 계획(kinodynamic planning), 실시간 계획(real-time planning)으로 이어지는 흐름을 완성한다. 첨부된 구조에서는 이 주제가 최적화 기반 계획(optimization-based planning)이 시작되기 전 Chapter 03의 마지막 항목으로 배치되어 있다. 실질적인 역할은 샘플링 기반 탐색을 로봇 모델, 계획 장면, 제약조건, 궤적 처리, ROS 2 제어 및 실행 가능한 매니퓰레이터 운동과 연결하는 것이다.
