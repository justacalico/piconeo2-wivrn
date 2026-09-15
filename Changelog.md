# [1.0.0]() (Unreleased)

- Adds smoothing to 6-dof users HMD. ([MR !231](https://gitlab.com/HttpAnimations/piconeo2-wivrn/-/merge_requests/231))
- Streaming UI no longer follows the HMD. ([MR !232](https://gitlab.com/HttpAnimations/piconeo2-wivrn/-/merge_requests/232))
- Update to wivrn server 26.9. ([MR !233](https://gitlab.com/HttpAnimations/piconeo2-wivrn/-/merge_requests/233))

# [Beta 1.1](https://github.com/openlyst/piconeo2-wivrn/releases/download/Beta-1.1-(Unreleased)/wivrn-pvr-Beta-1.1-.Unreleased.apk)

- Reimplements Eye tracking
- Adds foveated streaming
- Adds controller models
- Ports the exact same prediction model that the official streaming assistant uses.
- Adds MIT-licensed submodule for license compliance (This is only for the ALVR shim)
- Fixed stereo IPD mismatch
- Fixed springing HMD tracking
- Fixed HMD shake by adding light EMA smoothing (tau=25ms) on head position only; controllers and orientation untouched
- Fixed passthrough warping at the edges of the view
- Fixed race conditions on stream lifecycle
- Fixed lobby status text mislabeling "Disconnected" as "Connecting"
- Fixed crashing when loading some Unity games like Anthro Heat.
- Fixed controllers not tracking when only one is connected
- Passthrough is now off by default
- Dynamic bitrate removed
- "Lower resolution for wireless" option removed
- Resolution slider now is %'ge

# [Beta 1.0](https://github.com/openlyst/piconeo2-wivrn/releases/download/Beta-1.0/wivrn-pvr-Beta-1.0.apk)

- Port back to PVR.
- Adds passthrough support.
- UI can now be moved freely.
- Removed broken rescale support.
- Update to wivrn server 26.6.2.
- Fixed UI stuttering on low power mode.
- Fix the client would crash clicking on pair.

# [RC6](https://files.catbox.moe/0dwyyf.apk)

- Add language support
- Use the headset OK button to click stuff in the UI when you don't have controllers connected
- Add eye tracking support for Pico Neo 2 Eye
- Add pupil dilation tracking
- Fix eye tracking breaking after taking the headset off and putting it back on
- Fix eye tracking dying if you close your eyes for half a second
- Fix audio stuttering
- Resolution can now change on the fly without reconnecting
- If the stream freezes for 2s the UI pops up, and after 5s it auto reconnects
- Fix floaty controller tracking
- Fix nausea from over-smoothed head tracking
- Fix IPD slider not doing anything
- IPD slider now steps in 0.5mm instead of 1mm
- Fix motion sickness from bad velocity prediction
- Reduce judder on network jitter by upping the decoder buffer to 4 frames
- Tracking uplink now runs at full 300Hz instead of 150Hz
- Render thread now reads target FPS from persist.pvr.config.target_fps (supports 90Hz)
- Improve video quality, less banding
- Reduce motion to photon latency from 34ms to 21ms
- Remove high power mode from settings (it didn't do anything)
- Fix USB connections failing sometimes
- Fix streaming UI not showing up when connecting from wivrn-dashboard
- Fix disconnect button not working on the connecting screen
- Fix showing reconnecting instead of disconnecting when the server drops
- Fix USB connect not working if you were already disconnected/reconnecting
- Lower default resolution to 1664x1756
- Resolution is now saved in your settings instead of being hardcoded
- Add option to lower resolution for wireless streaming
- Add dynamic bitrate that adjusts to your network
- Add dynamic bitrate toggle in settings (on by default)
- Settings page is now the same whether streaming or in lobby

# [RC5](https://gitlab.com/HttpAnimations/piconeo2-wivrn/-/tree/9d99acc0dcf0d58008930e10f6eda1fcabde2896)

 - Fix lobby flickering if your battery was low
 - Fix lobby resolution scaling
 - Fix trigger clicking the wrong settings item (was offset 35px too high)
 - Fix trigger slightly too high when activating in settings.
 - Improve stuttering.
 - Set default bitrate to 50.
 - Add bitrate slider to settings.
 - Harden USB pairing and disconnection handling.
 - Fix disconnecting causing the app to hang/crash.

# [RC4](https://gitlab.com/HttpAnimations/piconeo2-wivrn/-/tree/13457b85ce3a656dd60ace3279cdf1a2d05f2313)

 - Harden rendering pipeline
 - Fix controller face buttons
 - Fix controller trigger buttons
 - Fix foveation causing apps to render upside down (sorry aus)
 - Doubles tracking frequency to 300Hz
 - Remove AV1
 - Adds the ability to change software IPD
 - Fixed sliders in settings
 - Streams at native 2048x2160 resolution
 - Resolution slider is now based on res's not %'s
 - Adds WiVRn icon
 - Fix close button not working in the lobby for pairing 
 - Servers are now remembered after pairing with autoconnect support
 - Adds support for boundaries
 - Adds support for microphone input
 - Harden pairing/connecting over usb
 
# RC3

- Fixes all stuttering and jitters 
- Fix the memory leak 

# RC2 

### New
- Port to openxr
- Fix tracking
- Fix height
- Ability to recenter
- Audio support
- Remove lobby floor

### Known bugs
- Holding down buttons can get stuck
- Stream 1 has a stutter however way less than the PVR version
- Pointer in lobby doesn't work you have to pair from the PC
- Beat saber might be flipped upside down depending on your pc

# RC1 (PVR)
- Initial release
