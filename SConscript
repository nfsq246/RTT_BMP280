from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

src += Glob('sensor_bs_bmp280.c')
src += Glob('lib/bmp280.c')

# add lsm6dsl include path.
path  = [cwd, cwd + '/lib']

# add src and include to group.
group = DefineGroup('bmp280', src, depend = ['PKG_USING_BMP280'], CPPPATH = path)

Return('group')
