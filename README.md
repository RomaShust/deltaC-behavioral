# deltaC-behavioral
Arduino protocols for deltaC behavioral experiments

This is Arduino code to run behavioral experiments in concentration changes discrimination.
Given version supports three levels of concentration: high (H), medium (M) and low (L)
Each experiment presented 4 concentration sequences for 2 sniff cycles
Concentration is changed between first and second sniff cycle on the inhalation offset trigger
    H-H and M-M are go trials
    H-M and M-L are no-go trial
    
When we run this paradigm without odorant but only clean air, mice were able to perform tast nearly perfect.
Hypothesis: there are transients in flow and pressure during no-go trials that mice can detect.
