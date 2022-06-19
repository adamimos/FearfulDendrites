@echo off

FOR /L %%A IN (150,5,300) DO (
FOR /L %%R in (1,1,20) DO (
  ECHO %%A
  start python batch_py_numsyn_tuft.py %%A
)
timeout 1200
)
