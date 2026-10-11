# Verified v2 native previews

Each GIF shows six seconds from the accepted full video at its original speed. All SUMO GUI panes are post-capture replay. The live-highway source separately has audited official two-way live acquisition.

## highway

![highway](highway.gif)

334 source frames, 16.7 s. Full release asset: `highway-carla-sumo-four-pane.mp4`.

## ramp-entry

![ramp-entry](ramp-entry.gif)

487 source frames, 24.35 s. Full release asset: `ramp-entry-carla-sumo-four-pane.mp4`.

## ramp-exit

![ramp-exit](ramp-exit.gif)

476 source frames, 23.8 s. Full release asset: `ramp-exit-carla-sumo-four-pane.mp4`.

## left-turn

![left-turn](left-turn.gif)

580 source frames, 29.0 s. Full release asset: `left-turn-carla-sumo-four-pane.mp4`.

SUMO representation qualification: one invisible background endpoint sample has an unchanged empty lane field. The original strict all-lane audit remains FAIL; ego lanes, exact poses/times and native images are verified.

## roundabout

![roundabout](roundabout.gif)

1001 source frames, 50.05 s. Full release asset: `roundabout-carla-sumo-four-pane.mp4`.

SUMO representation qualification: two visible ego endpoint lane fields and 14 invisible background lane fields remain empty. All-actor and all-ego lane membership are false; the original strict FAIL remains. Exact poses/times and all native source PNG hashes are verified.

## live-highway

![live-highway](live-highway.gif)

334 source frames, 16.7 s. Full release asset: `live-highway-carla-sumo-four-pane.mp4`.

Release URLs remain pending until actual upload and remote SHA verification. See preview-index.json and accepted-v2-summary.json for hashes and scope.
