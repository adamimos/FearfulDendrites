@echo off

FOR /L %%R in (1,1,15) DO (
  ECHO %%A
  start python batch_py_numsyn_basal_example.py 235
)

FOR /L %%R in (1,1,15) DO (
  ECHO %%A
  start python batch_py_numsyn_tuft_example.py 235
)
