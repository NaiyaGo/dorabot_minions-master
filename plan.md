Scheme 1 — Simulator Rollout + Congestion Filtering (best starting point)
Run the simulator headless at scale, auto-detect when agents get stuck (velocity ≈ 0 while destination unreached), and record the state-action window around each event. Use MA-RRT* or iNash as the expert oracle. Fully automated and scalable — yields ~5–15 congestion events per 60-minute run.

Scheme 2 — Scripted Congestion Injection (best for coverage)
Programmatically construct specific deadlock topologies: head-on collisions, T-junction conflicts, port funnels, circular deadlocks. Generate 100+ variants per type with randomized parameters. Gives you controlled, reproducible, edge-case-covering data that Scheme 1 might never organically produce.




Recommended combination: Start with Scheme 2 (scripted scenarios, 500–1000 samples) + Scheme 1 (organic events from long runs, adds distribution diversity). Optionally re-label a subset with Scheme 5 for quality improvement.

The plan details the exact files to modify in the simulator, the data format per sample (visual frames, LIDAR arrays, agent states, language strings, actions), and the full pipeline architecture.