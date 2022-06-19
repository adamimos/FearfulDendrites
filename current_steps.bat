@echo off

FOR /L %%A IN (-100,10,350) DO (
  ECHO %%A
  start python current_steps.py %%A
)
