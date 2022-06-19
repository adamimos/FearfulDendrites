@echo off

FOR /L %%R in (1,1,15) DO (
  ECHO %%A
  python batch_py_numsyn_tuft_example.py 245
)
