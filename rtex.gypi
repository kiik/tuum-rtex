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
    '--std=c++11',
    '<!@(pkg-config --cflags aruco)',
  ],
  'libraries': [
    '<!@(pkg-config --libs aruco)',
  ],
  'include_dirs': [
    'inc'
  ],
  'sources': [
    'src/cmv/conv.c',
    'src/cmv/cmvision.cpp',
    'src/cmv/blob_finder.cpp',
    'src/cmv/object_search.cpp',
    'src/cmv/marker_search.cpp',
    'src/rtx_cmv.cpp',

    'src/rtx_ctl.cpp',
    'src/rtx_cmd.cpp',
    'src/rtx_init.cpp',
    'src/robotex.cpp',

    'src/game/rtx_football.cpp',
    'src/game/rtx_team_football.cpp',
    'src/game/rtx_basketball.cpp',

    'src/core/rtx_GameField.cpp',
    'src/core/rtx_LogicManager.cpp',
    'src/core/rtx_RobotexCommSrv.cpp',

    'src/protocol/rtx_DriveProtocol.cpp',
    'src/protocol/rtx_HardwareProtocol.cpp',
    'src/protocol/rtx_VisionProtocol.cpp',
  ],
}
