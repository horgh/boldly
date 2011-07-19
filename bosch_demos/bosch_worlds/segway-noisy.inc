include "sick.inc"

define segway position
(
  color "gray"
  size3 [0.685  0.79 1.75]
  origin3 [-0.2 0 0]
  gui_nose 1
  drive "diff"

  localization "odom"
  odom_error [0 0.03 0 0.0 0.01 0.03]
  # For no error, use this
  #localization "gps"

  blocks 1
  block[0].points 8
  block[0].point[0] [ -0.2825 -0.395 ]
  block[0].point[1] [  0.2825 -0.395 ]
  block[0].point[2] [  0.2825 -0.2   ]
  block[0].point[3] [  0.4025 -0.2   ]
  block[0].point[4] [  0.4025  0.2   ]
  block[0].point[5] [  0.2825  0.2   ]
  block[0].point[6] [  0.2825  0.395 ]
  block[0].point[7] [ -0.2825  0.395 ]

  sick(pose3 [0.2 0.0 0.67])
)
