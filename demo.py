"""Self State Aware Autonomy for Disconnected Driving
Wonsik Choi and Jeongin Choi
janefather@gmail.com; cjeongin024@gmail.com
Illustrative reference model; Python 3.10+ standard library.
"""
import argparse,heapq,json,math
from dataclasses import dataclass,field

@dataclass
class Ledger:
    energy_wh: float
    reserve_wh: float
    revision: int = 0
    records: list = field(default_factory=list)

    def event(self, kind, **details):
        self.records.append(dict(kind=kind, revision=self.revision, **details))

    def update(self, **details):
        self.revision += 1
        self.event('OBSERVATION_UPDATE', **details)

    def execute(self, name, task_wh, overhead_wh, revision, valid=True):
        # In this synchronous example, validation and commit share a revision.
        cost = task_wh + overhead_wh
        allow = valid and revision == self.revision and self.energy_wh-cost >= self.reserve_wh
        self.event('COMMIT' if allow else 'DENY', action=name,
                   task_wh=task_wh, overhead_wh=overhead_wh,
                   before_wh=self.energy_wh,
                   after_wh=self.energy_wh-cost if allow else self.energy_wh,
                   valid=valid, proposed_revision=revision)
        if allow:
            self.energy_wh -= cost
        return allow

def route(start, goal, blocked, width=5, height=5):
    """A* builds a route from the current map, without prewritten routes."""
    dist={start: 0}; parent={}; queue=[(0, 0, start)]
    while queue:
        _, cost, p=heapq.heappop(queue)
        if cost != dist[p]:
            continue
        if p == goal:
            path=[p]
            while p != start:
                p=parent[p]; path.append(p)
            return list(reversed(path))
        for dx,dy in [(1,0),(0,1),(-1,0),(0,-1)]:
            n=(p[0]+dx,p[1]+dy)
            if not (0 <= n[0] < width and 0 <= n[1] < height) or n in blocked:
                continue
            nc=cost+1
            if nc < dist.get(n, math.inf):
                dist[n]=nc; parent[n]=p
                h=abs(goal[0]-n[0])+abs(goal[1]-n[1])
                heapq.heappush(queue,(nc+h,nc,n))
    return None

def driving_case(initial_energy):
    led=Ledger(initial_energy,4.0)
    start=(0,2);destination=(4,2);blocked=set()
    initial=route(start,destination,blocked)
    old_revision=led.revision
    led.event('PROPOSE',route=initial,task_wh=4.0,overhead_wh=0.5)
    # Connectivity loss does not itself invalidate onboard perception.
    # A concurrent, independently sensed road closure invalidates the route.
    blocked.add((2,2))
    led.update(network='unavailable',blocked_cell=(2,2),onboard_perception='valid')
    old_accepted=led.execute('stale_route',4.0,0.5,old_revision,False)
    revised=route(start,destination,blocked)
    steps=len(revised)-1
    led.event('PROPOSE',route=revised,terminal='destination',task_wh=float(steps),overhead_wh=0.5)
    dest_ok=led.execute('offline_destination_route',float(steps),0.5,led.revision,True)
    selected=destination;selected_route=revised
    if not dest_ok:
        # Candidate refuges are inputs; the selected route is computed online.
        candidates=[]
        for refuge in [(1,1),(3,1)]:
            path=route(start,refuge,blocked)
            if path:
                cost=float(len(path)-1)+0.5
                if led.energy_wh-cost >= led.reserve_wh:
                    candidates.append((cost,refuge,path))
        if candidates:
            cost,selected,selected_route=min(candidates)
            led.event('PROPOSE',route=selected_route,terminal='refuge',task_wh=cost-0.5,overhead_wh=0.5)
            accepted=led.execute('offline_refuge_route',cost-0.5,0.5,led.revision,True)
        else:
            selected=None;selected_route=None;accepted=False
            led.event('OPEN',reason='no_model_certified_terminal_route')
    else:
        accepted=True
    control_ok=led.execute('essential_control_one_interval',0.2,0.0,led.revision)
    self_state=dict(owned_equipment=['onboard_perception','onboard_planner','vehicle_controller'],
                    unavailable_services=['network'],onboard_perception='valid_in_grid_model',
                    remaining_energy_wh=round(led.energy_wh,6),reserve_wh=led.reserve_wh,
                    selected_terminal='destination' if dest_ok else ('refuge' if selected else 'OPEN'),
                    unresolved=['lane_and_braking_dynamics','continuous_collision_avoidance'],
                    evidence_revision=led.revision)
    led.event('SELF_STATE',**self_state)
    return dict(initial_energy_wh=initial_energy,network='unavailable',
                old_route_accepted=old_accepted,destination_route_accepted=dest_ok,
                selected_terminal='destination' if dest_ok else ('refuge' if selected else 'OPEN'),
                selected_cell=selected,selected_route=selected_route,
                selected_route_accepted=accepted,essential_control_accepted=control_ok,
                final_energy_wh=round(led.energy_wh,6),reserve_wh=led.reserve_wh,
                self_state=self_state,ledger=led.records)

def driving_example():
    return dict(example='network_disconnected_autonomous_vehicle',inputs='illustrative',
                scope='high-level grid route planning; no lane, speed, braking or collision dynamics',
                normal_budget=driving_case(20.0),limited_budget=driving_case(10.0))

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',default='results.json')
    args=p.parse_args()
    result=driving_example()
    cases=[result['normal_budget'],result['limited_budget']]
    for case in cases:
        for event in case['ledger']:
            if event['kind']=='COMMIT':
                assert event['after_wh'] >= case['reserve_wh']
                assert abs(event['before_wh']-event['after_wh']-event['task_wh']-event['overhead_wh']) < 1e-9
            if event['kind']=='DENY':
                assert event['before_wh']==event['after_wh']
    assert result['normal_budget']['selected_terminal']=='destination'
    assert result['limited_budget']['selected_terminal']=='refuge'
    assert result['normal_budget']['final_energy_wh']==13.3
    assert result['limited_budget']['final_energy_wh']==7.3
    output={'title':'Self State Aware Autonomy for Disconnected Driving','version':'0.1','date':'2026-10-13','authors':[{'name': 'Wonsik Choi', 'email': 'janefather@gmail.com', 'orcid': '0009-0001-4263-9772'}, {'name': 'Jeongin Choi', 'email': 'cjeongin024@gmail.com'}],
            'method':'deterministic state based planning with explicit self state and resource gate',
            'performance_claim':'illustrative execution only; not measured hardware or learned AI advantage',
            'checks':'passed','result':result}
    with open(args.output,'w',encoding='utf-8') as f:
        json.dump(output,f,ensure_ascii=False,indent=2)
    print('Execution and invariants passed: '+args.output)

if __name__=='__main__':
    main()
