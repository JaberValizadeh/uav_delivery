# uav_delivery

Three **hector quadrotors** collect and deliver parcels across a **32 × 32 m
city block grid** in Gazebo, on ROS Noetic.

Every parcel starts at **one depot in the corner** of the city and goes out to a
**customer address drawn at random** — hub and spoke, the way a real delivery
operation works. A single dispatcher assigns every box to **exactly one**
aircraft; the courier flies the streets between fourteen towers on a
mast-mounted laser, descends onto its box, welds it on, carries it out to the
customer, sets it down on the address and climbs away. One command:

```bash
source ~/uav_ws/src/uav_pick_place/setup_uav.bash
roslaunch uav_delivery delivery.launch
```

Add `gui:=false` to run it headless, which is how it was tested, and
`rviz:=true` to watch the whole board in one view.

To launch the simulation with RViz showing the depots, routes and aircraft:

```bash
roslaunch uav_delivery delivery.launch rviz:=true
```

---

## One box, one drone

That guarantee lives in one field — `Parcel.carrier` — and in the rule that the
assignment is **sticky**.

The sibling package `uav_bushfire` recomputes nearest-first tasking every tick
and retasks freely, and that is right for fires: a fire is a *place*, and
abandoning one to fly at a nearer one costs nothing but the flying. A parcel is
a **commitment**. Halfway through the job the drone is holding the thing, and
there is no useful sense in which another aircraft takes over. So:

* a parcel is assigned once, while it is on the ground and the courier is
  empty;
* the pairing then holds until the box is confirmed down at its address;
* the only way a courier gives a job back is to say so out loud — a descent
  that will not converge, a failed weld, or being respawned while loaded.

Idle drones are matched to waiting parcels greedily by distance to the
**pickup**: score every (idle drone, waiting parcel) pair, take the shortest,
drop both, repeat. Not optimal, but it never hands one parcel to two aircraft,
which is the property the scenario actually requires.

## What counts as delivered

**Where the box actually is**, read from `/gazebo/model_states` — not what the
courier believes. The courier's release report is only permission to go and
look.

That distinction pays for itself the moment anything goes wrong. A courier that
releases early, gets bumped on the way down, or lets go over the wrong depot
believes it has delivered; the box says otherwise. So a misdelivery is not an
error case needing special handling — the parcel returns to `WAITING` with its
**source updated to wherever the box came to rest**, and the next idle drone is
sent to fetch it from there. A box knocked into open ground is simply a new
pickup job whose source is "open ground".

```
dist(box, dst) <= deliver_radius   AND   box_z <= grounded_z
        (0.45 m)                              (0.12 m)
```

## How it works

```
                 ┌── /delivery/parcels ─────────> pose_delivery_monitor (logs)
dispatcher ──────┼── /delivery/markers ─────────> RViz
 (parcel book,   │
  assignment,    └── /uav_N/delivery/task ──────> courier ──/cmd_vel──> controller/twist
  adjudication)                                      │  ▲        ▲
       ▲                                             │  │       scan
       ├──── /delivery/report ───────────────────────┘  │
       ├──── /uav_N/ground_truth/state ─────────────────┘
       └──── /gazebo/model_states  (where the boxes really are)

                       courier ──> /link_attacher_node/{attach,detach}
```

| Node | Scope | Role |
|---|---|---|
| `dispatcher` | **global** | Owns the parcel pool, spawns and moves the boxes, draws the addresses, assigns exclusively, adjudicates delivery, re-issues a delivered slot |
| `courier` | per drone | `TAKEOFF` → `ALIGN` → `CHASE` → `DESCEND` → `ACT` → `CLIMB`, run **twice**: once to the pickup, once to the address |
| `pose_delivery_monitor` | per drone | Passive. Logs pose, the job in hand, and the state of the board |

### One task, two legs

Everything in the state machine runs twice per parcel. A fire or a treasure is
one place the aircraft has to reach; a parcel is **two, in order**, and the
second only becomes reachable by doing something at the first. So the courier
carries a *leg* alongside its phase — `PICKUP`, then `DELIVERY` — and
`ALIGN`/`CHASE`/`DESCEND`/`ACT` are the same code pointed at a different goal
with a different action at the bottom. A phase per leg would double the machine
to say the same thing twice.

The legs are not symmetrical in one respect, and it is the one that bites: on
the pickup leg the drone is empty and the thing it is descending onto is
**loose**, so a sloppy approach knocks the box away; on the delivery leg it is
carrying 0.04 kg of inertia the controller cannot see, so the same approach is
flown by a slightly different aircraft than the gains were tuned for.

### The descent is gated

`CHASE` ends *overhead* at cruise altitude, because neither end of the job can
be flown to — the aircraft would have to land on it. `DESCEND` then rotates the
world-frame position error into the body frame and commands **both**
`linear.x` and `linear.y`: proper station-keeping, and the only place this node
uses lateral velocity.

Unlike the bushfire version, the descent is **gated on being centred**. A water
drop is over when the aircraft is low enough; a grasp is over when the pad is
on the box, and the pad is 0.09 m across against a 0.10 m box. So `DESCEND`
comes down only while the horizontal error is inside `grasp_tolerance`
(0.06 m), and actively holds a hover altitude 0.20 m above the box when it is
not. Sinking onto a box you are 5 cm off is how you knock it across the yard.

Everything vertical is derived from one number:

```
work_altitude = box_size + grasp_reach = 0.10 + 0.21 = 0.31 m
```

which puts the pad exactly on top of a box resting on the ground — and, on the
delivery leg, puts a carried box's underside exactly on the ground, so it is
*set down* rather than dropped.

## Staying out of each other's way

Three aircraft sharing a yard need more than the laser, and the reason is
geometric rather than a matter of tuning. **The scanner is a horizontal plane
on a mast**: it sees another drone beautifully when the two are at the same
height and cannot see it at all when one is above or below. That is precisely
the geometry of the encounters that matter here — one aircraft descending onto
a depot while another crosses over the top of it, or two climbing away from
adjacent depots into the same piece of sky.

So deconfliction is done twice.

**Structurally**, each courier transits on its own altitude layer:

```
uav_2  ────────────────────────────  2.5 m
uav_1  ────────────────────────────  1.9 m
uav_0  ────────────────────────────  1.3 m
                                     0.31 m  ← work altitude, shared
```

An aircraft occupies about 0.56 m vertically — mast top at +0.25, a slung box
at −0.31 — so a 0.6 m gap means two *transits* cannot overlap at all, and most
of a run is transit. This is why the crates are 3 m tall, as tall as the walls:
the top layer puts a drone at 2.5 m with its mast at 2.75, and a 2.5 m crate
would let that one sail over the whole obstacle field and never use its laser.

`separation_z` **must stay below** `cruise_layer_gap`, and getting that backwards
is not a subtle failure. At 0.9 against a 0.6 m gap, two aircraft on adjacent
layers counted as in conflict every time one passed over the other — correctly
separated by design and yielding to each other anyway. The layers stopped doing
anything and the fleet spent the run stopping.

**Serialised descents.** A descent is a move through *every layer underneath
you*, so it is the one manoeuvre the layers cannot protect. A courier will not
start down while another aircraft is anywhere below it near its goal; it holds
station on its own layer — where it is separated from everyone by construction
— and queues. Checking only for aircraft already down *in* the depot is not
enough, and that was measured: a drone on the top layer started down through the
middle layer exactly where another was transiting, and the two passed within
0.09 m before either noticed.

**Reactively**, for whatever is left, each courier subscribes to the other
aircraft's odometry and watches them directly. Inside `separation_xy` (1.5 m)
*and* `separation_z` (0.5 m), one of the two gives way — by **backing away**
along the line between them, not by stopping.

Stopping sounds like the polite thing and does not work: the aircraft with right
of way is still flying, and if its goal lies beyond the one that stopped, it
flies straight into it. Measured — a pair that yielded at 0.32 m closed to
0.08 m anyway, because only one of them was doing anything. Retreating makes
the gap open regardless of what the other chooses to do, which is the property a
collision-avoidance rule actually needs.

Which one yields has to be decided identically by both, or they either both
stop forever or neither does. The rule is total:

1. a **loaded** aircraft has right of way over an empty one — it is the one
   whose loss costs a parcel, and the less manoeuvrable of the two;
2. between two in the same state, the **lower namespace** wins.

Both inputs are shared knowledge — positions come from odometry, and who is
carrying comes off the dispatcher's parcel board that every courier already
receives — so the two always reach opposite conclusions and there is no
negotiation to fail.

Yielding holds a commanded **altitude**, not a commanded zero rate, for the
same reason the descent does: `linear.z = 0` is "stop climbing", and a drone
giving way for ten seconds would otherwise sink ten centimetres.

The dispatcher watches all three and logs the closest approach the fleet ever
comes to itself, so the claim is measured rather than assumed.

## The airframe

hector's quadrotor with **both** a laser on a mast above it and a grasp pad
underneath — the mast from `uav_bushfire`, the pad from `uav_pick_place`. Both
of those packages warn at length about payload inertia, so it is worth saying
why carrying both is safe.

The controller reads mass and moment of inertia from Gazebo **exactly once**,
when it loads, and is never told anything afterwards. The distinction that
matters is between mass present at load time and mass that arrives later:

| | mass | offset | added `m·d²` | vs airframe `0.01152` | seen by controller? |
|---|---|---|---|---|---|
| laser + mast | 0.28 kg | +0.25 m | 0.0169 | **+147 %** | **yes** — in the URDF, lumped into `base_link` |
| a carried box | 0.04 kg | −0.26 m | 0.0027 | +23 % | **no** — welded at runtime |

So the heavy thing above the aircraft is harmless and the light thing below it
is the constraint — the opposite way round from what the masses suggest. The
controller multiplies its attitude PID output by the inertia it read, so a
bigger *known* inertia just means proportionally bigger torque and the same
response. A box is invisible, and `uav_pick_place` established the ceiling the
hard way: 0.15 kg on a longer stem was 142 % of the airframe's own inertia and
produced a sustained ±3° roll limit cycle that never decayed and timed out
every waypoint.

Thrust is not the constraint and never was: `force/z` caps at 30 N against a
14.5 N airframe, so there is about **1.5 kg of lift spare and none of it
usable**.

The gripper itself is `gazebo_ros_link_attacher` — a fixed joint welding the
box to `base_link`. Aerial grasping is not arm grasping: an arm can brace
against a table to close a hand, a drone has nothing to push against, so real
aerial manipulators grab on contact and hold through a near-rigid connection.
A fixed joint is the honest simulation of that. (Stock Noetic's vacuum gripper
plugin was rejected in `uav_pick_place`: it applies a *force*, which under a
free-flying base is a spring between two bodies that the position controller
fights.) The weld goes to `base_link` rather than `gripper_link` because Gazebo
folds fixed-joint children into their parent, so there is no `gripper_link`
body in the simulator to find.

## The city

Fourteen towers, 4 to 12 m tall, on a **9 m street grid** with 5.4 m
carriageways, one park, and the delivery depot in the south-west corner.
Everything is generated:

```bash
rosrun uav_delivery make_city            # defaults: seed 3
rosrun uav_delivery make_city --seed 7
```

which writes **two files from one pass of one loop**:

| | |
|---|---|
| `worlds/city_block.world` | what Gazebo shows the drones |
| `config/city_obstacles.yaml` | where the dispatcher may not put an address |

That pairing is the point. `delivery.launch` used to carry the obstacle list
inline as eight hand-typed triples, under a comment saying *"keep in step with
the world — nothing checks"*. With eight crates on a symmetric ring that was
survivable; with fourteen buildings it is not, and the failure mode is not a
crash but a customer address inside a wall, which looks for all the world like
a courier that cannot land. The launch now loads the generated file, so a
building cannot exist in Gazebo without the dispatcher knowing not to deliver
into it.

### Street width is not a cosmetic choice

The dispatcher rejects any address closer than `building_radius + 1.6 m` to a
building, because a box that comes to rest against a wall is inside the
descending courier's laser stop distance and costs a rescue rather than a
delivery. For square buildings of side `s` the circumscribed radius is
`s·√2/2`, and the middle of a street is `BLOCK_PITCH/2` from the towers either
side, so the whole layout reduces to one inequality:

```
BLOCK_PITCH / 2   >   s · √2/2  +  1.6
      4.5         >     2.26    +  1.6      ✓  margin 0.64 m
```

Narrow the streets or fatten the buildings and the city quietly becomes
undeliverable. So `make_city` does not trust that arithmetic — it samples the
customer region against the buildings it actually wrote and reports what it
found:

```
city: 14 buildings on a 9.0 m grid, 5.4 m streets
  deliverable address space: 62.4% of the customer region
```

### Trees, and what a 2D laser can see

Ten trees: a clump on the park lawn and street trees on the kerb line, one per
alternate block. Three numbers decide whether a tree is an obstacle the
couriers handle or one they fly into, and none of them are aesthetic.

**Trunk thickness.** The Hokuyo is 181 rays over 270°, so 1.5° apart. A 0.10 m
ornamental trunk at 4 m subtends 1.4° and can fall clean between two rays.
These are 0.24 m, which is 3.4° at 4 m and 11° at the 1.6 m avoidance range —
four to seven rays. A thin trunk is the one shape this sensor can miss.

**Trunk continuity.** The scan plane sits 0.25 m *above* `base_link`, and
nothing on the airframe is above +0.04, so the laser always looks slightly over
the top of what the aircraft can hit. That is safe for a solid object and unsafe
for one with a gap at scan height — the classic 2D-lidar table-leg problem. A
tree is safe because the trunk runs unbroken from the ground to the canopy.

**Canopy base**, at 2.40 m, is under the top cruise layer and over the middle
one. So `uav_2` meets foliage head-on and sees it — its scan at 2.85 m is inside
the leaves — while `uav_0` and `uav_1` pass beneath and only ever see trunk.
That is deliberate: it gives the top layer a real obstacle rather than three
layers all solving the same trunk.

Set on the kerb, a tree is 2.7 m from the street centreline against avoidance
that triggers at 1.6 m, so a courier tracking the middle of a street never
reacts to a tree it is merely passing. The trees cost deliverable address space,
which is the price of them being real obstacles: **62.4% → 49.9%**.

### Checking that they are actually avoided

"The drones avoid obstacles" used to be a claim nothing here could check — the
evidence was that nobody watched one hit a building, which is the same evidence
you get from not watching at all.

`clearance_monitor` measures it, against the exact collision geometry
`make_city` writes alongside the world. It reports the smallest gap each
aircraft ever leaves:

```
  === clearance ================================================
   solids in the city  : 28
   contact threshold   : 0.35 m (rotor tip), near miss 0.55 m
   uav_0  closest 0.66 m at (-11.9, -6.5, 1.2)  contacts 0  near 0
   uav_1  closest 0.59 m at (  6.5, -2.9, 1.8)  contacts 0  near 0
   uav_2  closest 0.66 m at (  6.3, -2.4, 2.4)  contacts 0  near 0

   no contacts and no near misses; worst gap 0.59 m
```

Which is what set `stop_dist`. At the inherited 0.9 m the same measurement gave
a worst gap of **0.47 m** against a 0.35 m rotor radius — no contact, but 0.12 m
of margin, which a gust or a heavier box would eat. The near miss was a building
corner, not a tree; the nearest tree at that moment was 5.4 m away.

The fix is *not* to raise `avoid_dist`, and that is the coupling worth knowing:
addresses are drawn at least `retrieve_clearance` (1.6 m) from every obstacle,
so an `avoid_dist` above 1.6 would put the doorstep itself inside the avoidance
zone and the courier would yaw away from the point it is trying to hold over.
`stop_dist` is where the headroom is — reversing 0.2 m earlier is well inside
`avoid_dist`, so it changes how avoidance finishes, not when it starts:

| `stop_dist` | worst gap | contacts | near misses |
|---|---|---|---|
| 0.9 (was) | 0.47 m | 0 | 1 |
| **1.1** | **0.59 m** | **0** | **0** |

It measures to the **true surface**, not to the conservative circles the
dispatcher uses — those circumscribe each building, so a drone passing a corner
diagonally would look closer than it is and a safe flight would score as a near
miss. Buildings are measured as rectangles, trees to the trunk, and only while
the aircraft is inside that object's height band: a drone at 2.6 m is not in
contact with a tower it is 8 m to the side of.

Contacts are **edge-triggered**. A drone parked 0.3 m from a wall is one
contact, not two hundred; counting samples would make a slow approach look far
worse than a fast one, which is backwards.

The four boundary walls are in the measured geometry but deliberately *not* in
the dispatcher's keep-out list — addresses are drawn well inside them and the
courier has its own geofence, but an aircraft that hits one has still crashed,
and a monitor blind to the walls would score that as a clean run.

### What is solid and what is paint

Only the **buildings, trees and boundary wall** have collision — 48 collision
bodies out of 218 models. Roads, kerbs, lane markings, zebra crossings, the
pavement pads and the park are flat visuals a couple of millimetres proud of
the ground. They are what makes it read as a city, and none of them can stop a
drone or trap a released parcel. It is also why the obstacle list stays exactly
as long as there are things to hit.

The towers are **tall and thin on purpose**. The couriers cruise at 1.2, 1.9
and 2.6 m in three stacked layers, and the invariant inherited from the crates
is that *no layer can climb over an obstacle* — that is what makes the laser
matter. Everything here is at least 4 m, so it still holds; making them 12 m
instead of 3 costs nothing and turns the streets into canyons the aircraft fly
along rather than a field of blocks they fly between.

## The warehouse and its customers

```
   ┌──────────────────────────────────────┐ 32 x 32 m
   │        ▪         ▪                   │
   │             ▪          F7            │  ▪   crate stacks, 3 m
   │   ▪    D9        ▪        ▪          │
   │             ▪         H2             │ A1-J10  3.2 m address grid,
   │      B8            ▪                 │         lettered W-E,
   │ [WH]                        C4       │         numbered S-N
   └──────────────────────────────────────┘
```

The warehouse is a **2.4 m apron in the far south-west corner** — small,
because it holds the outgoing parcels and nothing else, and every square metre
of it is airspace three aircraft have to share. It is deliberately nowhere near
the customers: every job is a real transit of **9 to 35 m** out and the same
back empty. An operation whose depot sits among its own customers is not really
a distribution problem.

**Customer addresses are drawn at random on every re-issue.** There are no
fixed pads. An address is a point anywhere inside `customer_half`, excluding a
`customer_min_from_warehouse` circle around the hub, kept `customer_gap` clear
of every other outstanding address and clear of the crates. The dispatcher
spawns a doorstep marker in the parcel's own colour at whatever point it drew,
so no two runs deliver to the same set of places and a reader cannot check the
drone's work against the source file.

Addresses are named by **grid reference** — `A1` to `J10`, 3.2 m cells lettered
west to east and numbered south to north — and the name is *computed from the
position* rather than stored beside it, so an address and the place it refers
to cannot drift apart. That is a real failure mode when a list of names and a
list of coordinates are maintained separately.

Three things disqualify a drawn point, and each is a failure that was cheaper
to design out than to handle:

| rejected if | because |
|---|---|
| too near a crate | a courier cannot descend onto a doorstep inside its own laser's stop distance without the safety filter backing it off at 0.8 m and tipping it over |
| too near the warehouse | the delivery would be over before the aircraft finished climbing — a score with no flying in it |
| too near another live address | two doorsteps closer than the descent guard's radius mean the second courier queues behind the first for nothing |

### Laying the boxes out on one apron

With a shared source, two things that used to be rare become constant.

**Boxes must not share a point.** Two dynamic bodies spawned inside each other
are two bodies Gazebo's contact solver flings apart at speed. Under the old
scheme that needed an unlucky draw — seen once, when all five parcels happened
to pick the same depot, and the run produced no pickups at all because every
courier was chasing a box that was still moving. With one source it would be
every run, so positions are sampled against a spacing constraint rather than
jittered and hoped over.

**The spacing also decides how many couriers can load at once.** A courier will
not descend while another aircraft is within `depot_guard_xy` (1.3 m) of its
goal, so boxes closer together than that turn the hub into a single-file queue.
The dispatcher checks at startup that the parcel count actually fits the apron
at the requested spacing and warns if it does not, rather than silently packing
them closer than asked.

### A dropped parcel can land somewhere unreachable

The recovery path had a hole in it, and the corner-warehouse layout is what
exposed it. A courier that is upset drops its box rather than dragging it
through a teleport, and the parcel used to go straight back on the board *at
whatever point the box came to rest*. If that point is against a crate, it is
inside the laser's stop distance: the next courier descends, the safety filter
backs it off and yaws it at 0.8 m, and it tips over. Assignment is
nearest-first and the drone that just dropped it is invariably the nearest, so
it goes back for the same box again.

Measured, before the fix: a box dropped **0.67 m from a crate stack** cost
**three rescues in ninety seconds**, the same aircraft each time, and that
parcel was never delivered.

The dispatcher is now told where the crates are, and a parcel that comes to
rest too close to one is **recovered to the warehouse** — the box is teleported
back to the apron and keeps its customer address. That is what a real operation
does with a parcel in a hedge: someone picks it up by hand rather than sending
the aircraft at it until something breaks.

### The couriers park by the warehouse

An empty aircraft belongs at the hub, so they start around the apron rather
than in the middle of the yard: 2.9–3.4 m off it so no parcel spawns
underneath one, at least 3.4 m from each other, and clear of the shed. They
cannot sit *on* the apron — a drone parked there is a drone with a box
appearing under it.

## Inherited hard-won behaviour

The flying comes from `uav_bushfire`, including everything its debugging
produced. In brief, and documented at length in `scripts/courier`:

* **`/cmd_vel` is body-frame**, not world-frame — `TwistController` rotates it
  by yaw, so a world-frame command flies the aircraft along `2 × yaw`.
* **Self-hit rejection is conditional on tilt**, because a blanket 0.55 m floor
  blinds the drone exactly at contact range (a crate it is touching reads
  0.33 m).
* **A respawn cuts the motors first** (`/shutdown`, teleport, settle,
  `/engage`), because that is the only thing that clears the attitude PIDs —
  without it, one upset became 53 rescues in 90 seconds.
* **Deflections are committed to** for `avoid_hold` after the front sector
  clears, or the drone re-aims the instant it is past and turns back in.
* **Gazebo models are moved, never respawned** — delete-then-spawn races
  Gazebo's deferred world update.

Two things this package added:

* **A courier drops its box before being rescued.** Teleporting one end of a
  fixed joint across the yard is not a state Gazebo has any reason to handle
  gracefully; and dropping it puts the parcel back on the board, where the
  dispatcher re-lists it from wherever it lands.
* **A parcel is only examined for delivery once its courier reports letting
  go.** See the first bug below.

## Three bugs this package produced

### 1. Every pickup read as a misdelivery, two milliseconds after the grasp

The obvious delivery test — "a carried box that is on the ground has been put
down" — is true of a box *the instant it is welded on*, because the aircraft
grasps it where it lies and has not climbed yet. Ungated, the parcel went back
on the board while the drone was still holding it, was immediately reassigned
to that same drone as a fresh pickup, and the two halves of the system spent
the run disagreeing about what was in the gripper. Four pickups, zero
deliveries.

The fix is the gate: a release report from the courier is what makes the
dispatcher look at all.

### 2. `linear.z = 0` is not "hold this height"

The descent holds a hover altitude while it centres and only comes down once
inside the grasp tolerance. The first version expressed "hold" as
`cmd.linear.z = 0.0`, which is wrong in a way that is invisible for about ten
seconds: `controller/twist` closes its loop on **velocity**, and nothing in the
stack closes one on position. Zero commanded velocity is not "stay here", it is
"stop climbing" — and the aircraft sinks underneath it at roughly a centimetre
a second.

So a courier that spent twenty seconds centring quietly gave away twenty
centimetres of altitude, dropped below the grasp window it was trying to hit,
and — since nothing brings it back up while it is still off-centre — was out of
the running permanently. Measured: three drones, **189 seconds in `DESCEND`
between them, and not one grasp**. The fix is to command an *altitude* for the
hover leg too, so the descent has two levels rather than one and off-centre
means "hold 0.51 m" rather than "stop climbing".

This is the same fact `uav_pick_place` records as needing `station_keep()`
between waypoints. It is worth re-learning here because the symptom is
completely different: there it was a drone drifting down between legs, here it
was a grasp that never triggered.

### 3. The log never reached the terminal — twice, for the same reason

roslaunch captures a node's output through a pipe, and Python **block-buffers**
into a pipe rather than line-buffering as it would to a terminal. The couriers
get away with it by printing a status line every second and constantly filling
the buffer; the dispatcher logs only on events, so minutes of assignments and
deliveries sat unflushed and the run looked like nothing was being despatched.
The whole first diagnosis had to be done from `~/.ros/log/*/dispatcher-*.log`.

`sys.stdout.reconfigure(line_buffering=True)` fixed it — and then the *same
bug* reappeared for warnings only, which is the more interesting half. rospy
sends INFO to stdout but **WARN and ERROR to stderr**, and on Python 3.8 stderr
into a pipe is block-buffered too; it only became unconditionally line-buffered
in 3.9. So fixing stdout alone repairs the ordinary log and silently keeps
exactly the lines worth reading — proximity, misdeliveries, rescues — in a
buffer. Caught while measuring the deconfliction A/B: **seven `PROXIMITY`
warnings in the node's own log file and none at all in the terminal**, which
would have made the "after" run look cleaner than it was. Both streams are
reconfigured now, in all three nodes.

## Arguments

`roslaunch uav_delivery delivery.launch gui:=false num_parcels:=7`

| Arg | Default | Meaning |
|---|---|---|
| `gui` | `true` | Gazebo client. `false` is roughly twice as fast |
| `rviz` | `false` | RViz showing depots, routes and all three aircraft |
| `num_parcels` | `5` | Parcels in the system. Keep it above the fleet size |
| `depots` | `A`…`H` | `[[name, x, y], …]`. **Keep in step with the world** |
| `box_size` | `0.10` | Box edge (m) |
| `box_mass` | `0.04` | **The tightest constraint in the package** — see the airframe |
| `grasp_reach` | `0.21` | Pad offset below `base_link`. **Must match the urdf** |
| `deliver_radius` | `0.45` | How close to the address a box must come to rest |
| `cruise_altitude` | `1.3` | Transit altitude of the **lowest** layer |
| `cruise_layer_gap` | `0.6` | Vertical spacing between the three transit layers. `0` puts everyone on one layer |
| `separation_xy` | `1.5` | Traffic conflict radius. `0` disables yielding |
| `separation_z` | `0.9` | ...and the vertical half of it |
| `arrive_radius` | `0.60` | Switch from flying to station-keeping inside this |
| `linear_speed` | `1.1` | Transit speed (m/s) |
| `avoid_dist` / `stop_dist` | `1.6` / `0.9` | Laser avoidance activation and full-stop ranges |

`cruise_layer_gap:=0 separation_xy:=0` turns both deconfliction mechanisms off,
which is how the before/after in **Verified** was measured — same binary, two
arguments.

## Verified

Four minutes headless, three couriers, five parcels. Deconfliction is switchable
from the command line, so both arms are the **same binary**:

```bash
# with            (default)
roslaunch uav_delivery delivery.launch gui:=false
# without
roslaunch uav_delivery delivery.launch gui:=false \
    cruise_layer_gap:=0 separation_xy:=0 depot_guard_xy:=0
```

Two runs per arm:

```
                          without deconfliction      with
                          run A3     run A4          run B5     run B6
ticks in conflict *          677        565               4         17
proximity warnings            28         19               1          2
rescues (upsets)               6          2               1          1
closest 3D approach         0.30 m     0.28 m         0.54 m     0.42 m
parcels delivered              9         12               6         15

* a control tick with two aircraft inside 0.80 m horizontally while
  within 0.50 m of each other vertically - i.e. genuinely overlapping,
  since the rotor span is 0.80 m
```

**Time spent in conflict falls by 97–99 %**, and it is the metric that matters:
it counts how long the fleet spends overlapping rather than whether it ever did.
Upsets fall with it.

Two things worth stating plainly rather than rounding off:

* **Throughput is unchanged.** Delivery counts vary widely run to run (6 to 15),
  and the means are the same either way — 10.5 both arms. Holding for an
  occupied depot cost almost nothing: 6 `HOLD` samples in a whole run.
* **The single closest approach is not reliably better** — B6 dipped to 0.14 m
  co-altitude, worse than either A run's worst moment. One instantaneous
  minimum over four minutes is a noisy statistic, and the honest claim is about
  sustained exposure, not about a guarantee. These are three reactive aircraft
  with no shared plan; they get out of each other's way far better than before,
  and they are not provably separated.

For the delivery task itself, from the earlier default-configuration run: 12
delivered from 15 pickups, **0 misdeliveries**, every box set down within 3 cm
of its address, and **no parcel ever claimed by more than one drone** — checked
by replaying the dispatcher's assignment, delivery and abandon lines in
timestamp order and tracking the claim set per parcel.

### Hub and spoke, measured

Three layouts, same code, four-and-a-half-minute runs:

```
                        point-to-point   central hub    corner hub,
                        (2 runs)         (14 m yard)    random addresses
delivered                  9 / 12             11              16
conflict ticks             4 / 17              0               0
proximity warnings         1 / 2               0               0
co-altitude closest    0.36 / 0.14 m       1.55 m          1.22 m
rescues                    1 / 1               5               0
abandoned parcels          2 / 2               5               0
misdeliveries              0 / 0               0               0
per-drone spread        4/1/6 · 5/4/6      12/1/4          6/6/4
```

Two results worth pulling out, because neither was the point of the change:

**Rescues and abandons went to zero.** The central-hub run lost five of each,
almost all of them one aircraft stuck retrieving a box that had been dropped
against a crate. Moving the warehouse to the corner took the crates away from
where the drops happen, and the unreachable-parcel recovery closed the loop
that made a single bad drop cost three rescues.

**The fleet balanced itself.** The central hub was badly lopsided — 12 / 1 / 4 —
because greedy nearest-first assignment compounds: whoever is nearest the hub
on release tends to win the next parcel too. With the hub in the corner and
spokes of 9–35 m, every aircraft returns to the same place and the round trip
dominates, so the advantage evaporates: 6 / 6 / 4 without touching the
assignment rule. Layout did what a scheduler would otherwise have had to.

## Known limitations

**A reactive controller has no plan.** It only avoids what it currently sees,
so a depot in a concave pocket can hold a courier up until the descent timeout
returns the parcel to the board.

**Deconfliction is mitigation, not proof.** Layers separate transits, the depot
guard serialises descents and the yield rule handles the rest, and together they
cut time-in-conflict by two orders of magnitude — but nothing here computes a
conflict-free trajectory, so two aircraft can still brush past each other. See
the caveat in **Verified**.

**The descent is blind downwards.** A horizontal laser says nothing about what
is below, so nothing prevents flying into the ground except `work_altitude` and
`min_altitude` being honest numbers — and here `min_altitude` (0.26 m) has to
sit *below* the working altitude rather than above it, which is why it is a
separate parameter.

**Nothing models the payload.** The controller flies the whole run believing it
is empty. That is what caps `box_mass`, and it is the interesting limitation —
lifting more means retuning `controller/twist` for the loaded case, or a
controller that is told what it is holding.

## Files

| | |
|---|---|
| `launch/delivery.launch` | Gazebo, the yard, three couriers, the dispatcher, optional RViz |
| `launch/one_uav.launch` | one aircraft's airframe + courier + monitor |
| `scripts/dispatcher` | the parcel book, assignment, adjudication |
| `scripts/courier` | the two-leg controller and the gripper |
| `scripts/pose_delivery_monitor` | passive logging |
| `msg/Parcel.msg` | id, box, src address, dst address, state, carrier |
| `msg/ParcelArray.msg` | the whole board |
| `urdf/quadrotor_courier.gazebo.xacro` | hector's quadrotor + laser mast + grasp pad |
| `scripts/make_city` | generates the world **and** the keep-out list, together |
| `worlds/city_block.world` | generated: 14 towers, 10 trees, streets, park, depot |
| `config/city_obstacles.yaml` | generated: buildings and trees as keep-out circles |
| `config/city_geometry.yaml` | generated: exact solids, for the clearance monitor |
| `scripts/clearance_monitor` | measures the real gap to every building and tree |
| `worlds/depot_yard.world` | the older bare yard with eight crate stacks |
| `rviz/delivery.rviz` | depots, routes and aircraft in the shared `world` frame |

## Building

This package lives in `~/uav_ws` alongside hector, `gazebo_ros_link_attacher`
and the other UAV packages, so nothing extra is needed to build it. If the
workspace is not built yet, see
[uav_pick_place's README](../uav_pick_place/README.md#installing-the-dependencies)
— none of the third-party trees are packaged for Noetic and the build is
two-stage.

```bash
source ~/uav_ws/src/uav_pick_place/setup_uav.bash
cd ~/uav_ws && catkin_make --pkg uav_delivery
```

`setup_uav.bash` is not optional here even at runtime: it puts
`~/uav_ws/devel/lib` on `GAZEBO_PLUGIN_PATH`, and without it the link attacher
never loads, `/link_attacher_node/attach` never appears, and the couriers say
so and refuse to grasp anything.

## Related packages

* [`uav_pick_place`](../uav_pick_place) — one drone, a fixed list of blocks and
  pads. Where the gripper and the payload-inertia limit come from.
* [`uav_bushfire`](../uav_bushfire) — three drones, shared ground targets,
  central tasking. Where the flying comes from.
* [`uav_voice_control`](../uav_voice_control) — the pick-and-place mission,
  flown by talking to it.
