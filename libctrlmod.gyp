{
  'targets': [
    {
      'target_name': 'tuum',
      'type': 'executable',

      'includes': [
        '../libtuum/build/common.gypi',
        './rtex.gypi'
      ],

      'libraries': [
        '-Lobj',
        '-lpthread',
        '-lboost_system',
        '-lboost_thread',
        '-lboost_program_options',
        '-lboost_coroutine',
        '-lboost_context',
        '-lboost_filesystem',
        '-lglut',
        '-lwebsockets',
      ],
    },
  ],
}
