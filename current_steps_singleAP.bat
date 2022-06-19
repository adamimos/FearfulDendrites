@echo off

FOR /L %%A IN (100,100,300) DO (
  ECHO %%A
  start python current_steps_singleAP.py %%A
)
