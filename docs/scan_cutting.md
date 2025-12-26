## Assumptions

- The gap between any two blocks is less than $360^\circ - 2 \cdot spread$, where spread is the
  maximum difference between the azimuths of any two channels in a block.

Cases that are violating the above assumptions are marked with ⛔ in the below tables and diagrams.

## Action definitions

| Action | Description                                    | Modified State               |
| ------ | ---------------------------------------------- | ---------------------------- |
| `Ti`   | Reset timestamp of buffer $i$                  | None                         |
| `Ei`   | Emit scan buffer $i$, and change active buffer | $buf \leftarrow buf_{1 - i}$ |

## Cutting in FoV

State definitions:

| State   | Condition                                   |
| ------- | ------------------------------------------- |
| `F0`    | $\forall c . c \in buf_0$                   |
| `C0->1` | $buf = buf_0 \land \exists c . c \in buf_1$ |

Analogously for `F1` and `C1->0`.

Transition actions:

| To \ From | `F0`     | `C0->1` |
| --------- | -------- | ------- |
| `F0`      | -        |         |
| `C0->1`   | `T1`     | -       |
| `F1`      | `T1, E0` | `E0`    |
| `C1->0`   |          | ⛔      |

Analogously for `F1` and `C1->0`.

![FSM-CutInFov](img/scan_cutting_fsm.drawio)

## Cutting at FoV End

State definitions:

| State   | Condition                                             |
| ------- | ----------------------------------------------------- |
| `O0`    | $\forall c . c \in buf_0 \land c \notin fov$          |
| `F0`    | $\forall c . c \in buf_0 \land \exists c . c \in fov$ |
| `C0->1` | $buf = buf_0 \land \exists c . c \in buf_1$           |

Analogously for `O1`, `F1`, and `C1->0`.

Transition actions:

| To \ From | `O0`     | `F0`     | `C0->1`  |
| --------- | -------- | -------- | -------- |
| `O0`      | -        |          |          |
| `F0`      | `T0`     | -        |          |
| `C0->1`   | `T0`     | -        | -        |
| `O1`      | `T1, E0` | `E0`     | `E0`     |
| `F1`      |          | `T1, E0` | `T1, E0` |
| `C1->0`   |          |          | ⛔       |

Analogously for `O1`, `F1`, and `C1->0`.

![FSM-CutAtFovEnd](img/scan_cutting_fsm.drawio)
