# Self State Aware Autonomy for Disconnected Driving

Wonsik Choi and Jeongin Choi · Independent Researchers · 13 October 2026 · v0.1

- Wonsik Choi: janefather@gmail.com; ORCID https://orcid.org/0009-0001-4263-9772
- Jeongin Choi: cjeongin024@gmail.com

Network loss and a newly observed road closure during autonomous driving.
The grid planner computes a new route or a feasible refuge route according to energy constraints.
Outputs: normal budget 13.3 Wh, limited budget 7.3 Wh.
Lane, braking, continuous collision dynamics and learned AI generalization remain untested.

## Reproduce

Python 3.10 or later; standard library only.

```bash
python demo.py --output results.json
```

The checks verify declared resource accounting and symbolic action preconditions.
All numerical inputs are illustrative. Elementary self awareness is operationally
defined as maintaining equipment ownership, current resources, fault state and
unresolved conditions when selecting actions. Subjective experience is outside
this execution study. This is a research proposal and reference example.

Complete English and Korean proposals are included. The source execution
contracts are credited in its reference section. No external flight or vehicle
control software is incorporated.

## Proposal outreach record

The outreach folder contains the authors' outbound proposals and sent timestamps. SENT records an outbound submission, not acceptance or endorsement by the recipient.

- 2026-10-13T17:32:23+09:00 — sales@x.ai

- 2026-10-13T17:31:44+09:00 — jiajixie@tesla.com

Copyright (C) 2026 Wonsik Choi and Jeongin Choi. Background-IP ownership remains with its existing rights holders. Publishing this proposal establishes no production license, funding award, advisory appointment, or signed collaboration agreement. Existing licenses, where applicable, continue to govern their covered materials.
