# P4 Home Runtime Patch

This patch aligns the home VDA5050 simulator, Fleet Adapter navigation graph,
and direction Arbiter around one simulation-only corridor from node 2101 to
node 2108.

It intentionally does not replace the default reconstructed Fleet Adapter map
or configuration. The P4 Adapter is selected with `docker-compose.p4.yml`.

The endpoint holding bays are logical points on the main lane. Replace this
minimal graph with the complete company navigation graph and off-lane physical
holding bays before field deployment.
