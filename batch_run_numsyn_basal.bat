@echo off

FOR /L %%A IN (150,20,270) DO (
FOR /L %%X in (1,1,1) DO (
FOR /L %%R in (1,1,20) DO (
  ECHO %%A
  start python batch_py_numsyn_basal.py %%A
)
timeout 1200
)
)


FOR /L %%A IN (150,20,270) DO (
FOR /L %%X in (1,1,1) DO (
FOR /L %%R in (1,1,20) DO (
  ECHO %%A
  start python batch_py_numsyn_tuft.py %%A
)
timeout 1200
)
)
