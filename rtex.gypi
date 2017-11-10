{
  'dependencies': [
    'libenv',
    'liblpx',
    'libhal',

    'libsystem',
    'libcomm',

    'libuser',
  ],

 'cflags': [
    '--std=c++11'
  ],
  'include_dirs': [
    'inc'
  ],
  'sources': [
    'src/rtx_ctl.cpp',
    'src/rtx_fb.cpp',
    'src/rtx_tfb.cpp',
    'src/rtx_cmds.cpp',
    'src/rtx_context.cpp',

    'src/rtex_init.cpp',
    'src/rtex.cpp',

    'src/rtx_GameField.cpp',
    'src/LogicManager.cpp',
    'src/RobotexCommSrv.cpp',

    'src/protocol/rtex_DriveProtocol.cpp',
    'src/protocol/rtex_HardwareProtocol.cpp',
    'src/protocol/rtex_VisionProtocol.cpp',
  ],
}
