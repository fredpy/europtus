# europtus
A simple europa plan executive with async updates 

This program is meant to be a simple executive based on similar ideas to trex but instead 
of requiring observations to occur at the time they observed it allows for them to be 
received and integrated at a later date. 

This specific feature allows to have plan execution on domains where communication with 
the execution targets -- say remotes AUVs -- are observable only intermitently for example
due to intermittent communication.

The program is also aimed specifically toward being integrated in lsts platform with 
specifically the use of IMC for communication and neptus as potential interface.

## Dependencies

The program so far depends on:
 * cmake: http://www.cmake.org
 * C++ boost libraries 1.47 or above: http://www.boost.org
 * europa-pso planner 2.6: https://github.com/nasa/europa
 * lsts Dune library: https://github.com/LSTS/dune
 
Expected future dependencies:
 * _(maybe)_ lsts Neptus: https://github.com/LSTS/neptus
