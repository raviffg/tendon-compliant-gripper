# tendon-compliant-gripper

3-finger underactuated gripper. One tendon per finger, two phalanges per
finger, torsional return springs at every joint. Tendon excursion couples to
joint angle through the pulley radii; spring stiffness resolves the
underactuated joint split.

## Build

    make            # static lib in build/
    make eval       # friction sweep, wrench closure and slip margin
    make clean

## Layout

- `src/tendon_coupling.cpp` excursion solver, coupling matrix, contact force split
- `scripts/grasp_stiffness_eval.py` mu sweep, closure and tendon tension headroom
- `data/flex_force_calibration.csv` bench capture, pull vs force

## Notes

Rev C geometry. Spring constants in the source are measured, not modelled.
Rev D redirect pulley is not handled yet, see TODO in the solver.


## Expanded project documentation

The `docs/` directory contains project-specific design notes, verification scenario specifications, operations guides, data contracts, configuration templates, safety notes, and project reference templates.

**Important:** added documents are planning and review aids. They do not claim that tests have been executed, that placeholder configurations are supported runtime profiles, or that this project is certified for physical/clinical/production deployment. Confirm interfaces and limits against implementation and measured evidence.
