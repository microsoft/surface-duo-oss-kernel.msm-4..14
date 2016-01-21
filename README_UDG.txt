USER DEFINED GESTURES
---------------------



--- SYSFS FILE NODES ---

The synaptics_dsx_gesture.c driver module exposes the following sysfs file nodes.

** engine_enable
   Purpose: enable/disable gesture engine
   Access: write only
   Write: '0' - disable gesture engine
          '1' - enable gesture engine

** template_detection
   Purpose: report gesture engine event
   Access: read only
   Read: '0' - no event
         '0x0f' - detection of valid gesture input in detection mode
         '0x10' - completion of gesture input processing in registration mode

** template_size
   Purpose: report size of gesture template
   Access: read only
   Read: size of gesture template

** template_max_index
   Purpose: report maximum index number of gesture template
   Access: read only
   Read: maximum index number of gesture template

** template_index
   Purpose: select gesture template
   Access: write only
   Write: index number of gesture template

** template_valid
   Purpose: query and set valid bit of gesture template
   Access: read and write
   Read: '0' - gesture template invalid
         '1' - gesture template valid
   Write: '0' - set gesture template invalid
          '1' - set gesture template valid

** template_clear
   Purpose: clear gesture template
   Access: write only
   Write: '1' - clear gesture template

** template_data
   Purpose: retrieve and set gesture template data
   Access: read and write (binary)
   Read: gesture template data
   Write: gesture template data
   Format: x - N 4-byte floating point values (N = size of gesture template)
           y - N 4-byte floating point values (N = size of gesture template)
           scaling factor - 4-byte floating point value
           number of segments - 1-byte unsigned integer

** registration_enable
   Purpose: enable/disable registration mode
   Access: write only
   Write: '0' - disable registration mode
          '1' - enable registration mode

** registration_begin
   Purpose: start/end iteration of gesture input during registration
   Access: write only
   Write: '0' - end iteration of gesture input
          '1' - start iteration of gesture input

** registration_status
   Purpose: report status of current iteration of gesture input during registration
   Access: read only
   Read: '0x01' - success
         '0x02' - error: mismatch aginst previous gesture input
         '0x03' - error: matches another gesture template
         '0x04' - error: index number out of range
         '0x0e' - error: input buffer overflow
         '0x0f' - error: more than one finger during gesture input

** detection_enable
   Purpose: enable/disable detection mode
   Access: write only
   Write: '0' - disable detection mode
          '1' - enable detection mode

** detection_index
   Purpose: report index number of matching gesture template
   Access: read only
   Read: index number of matcing gesture template

** detection_score
   Purpose: report match score of gesture input against matching gesture template
   Access: read only
   Read: match score [0 : 100]

** trace_size
   Purpose: report size of detected gesture input
   Access: read only
   Read: size of detected gesture input

** trace_data
   Purpose: retrieve data of detected gesture input
   Access: read only (binary)
   Read: gesture input data
   Format: x - N 2-byte unsigned integers (N = size of detected gesture input)
           y - N 2-byte unsigned integers (N = size of detected gesture input)
           segment - N 1-byte unsigned integers (N = size of detected gesture input)



--- OPERATIONS ---

The following are sample operation flows based on the usage of the sysfs file nodes.

** Registration
   1) write '1' to engine_enable
   2) write '1' to registration_enable
   3) write index number of gesture template to template_index
   4) write '1' to registration_begin
   5) input gesture for registration
   6) write '0' to registration_begin
   7) poll template_detection for '0x10'
   8) read registration_status for result of gesture input
   9) goto 4) if more iterations required (3 consecutive successful iterations required)
   10) write '0' to registration_enable
   11) read template_data for template data of registered gesture
   12) write '0' to engine_enable

** Detection
   1) write '1' to engine_enable
   2) write '1' to detection_enable 
   3) input gesture
   4) poll template_detection for '0x0f'
   5) read detection_index for index number of matching gesture template
   6) read detection_score for match score against matching gesture template
   7) read trace_size for size of gesture input
   8) read trace_data for data of gesture input
   9) write '0' to detection_enable
   10) write '0' to engine_enable

** Write Gesture Template Data
   1) write index number of gesture template to template_index
   2) write gesture template data to template_data
   3) write '1' to template_valid

** Read Gesture Template Data
   1) write index number of gesture template to template_index
   2) read template_data for gesture template data

** Clear Gesture Template
   1) write index number of gesture template to template_index
   2) write '1' to template_clear

** Query Gesture Template
   1) write index number of gesture template to template_index
   2) read template_valid



--- POWER MANAGEMENT ---

The synaptics_dsx_gesture.c driver module is set up to automatically put the touch
controller in detection mode upon entering suspend. During suspend, when a gesture
input is detected and determined to be matching one of the valid registered gesture
templates, the driver module proceeds to wake up the system.

After the system wakes up, a return value of '0x0f' from reading the 'template_detection'
sysfs file node indicates that it was a matching gesture input that woke up system.
The index of the gesture template that produced the highest match score can be read from
the 'detection_index' sysfs file node, and the match score itself can be read from the
'detection_score' sysfs file node.
