# Subsystems

* LightShow
  * Outputs: Addressible RGB LED Strip
  * Sensors: USB Camera
  * Actions:
    * chooseLightShowFromCamera()
      * read the apriltag and set the internal show number
    * periodic()
      * look at the internal show number
      * look at the elapsed time
      * render a frame of the light animation

        EXAMPLE:
          0[*         ]
          1[ *        ]
          2[  *       ]
          3[   *      ]
          4[    *     ]
          5[     *    ]
          6[      *   ]

* ChurroDispenser
  * Outputs: Falcon Motor
  * Sensors: Falcon Encoder
  * Actions:
    * moveChurroOut()
      * drive the motor forward @ 30%
    * startChurro()
      * reset encoder to zero
    * churroIsOut() -> bool
      * when encoder > 10000 (or whatever the number is)

* ChickenScreamer
  * Outputs: Talon SRX (which drives an air pump)
  * Sensors: none
  * Actions:
    * screamQuietly()
      * run the Talon SRX @ 50%
    * screamSuperLoud()
      * run the Talon SRX @ 100%
    * shutUp()
      * run the Talon SRX @ 0%
