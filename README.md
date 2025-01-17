# motorMicos
EPICS motor drivers for the following [Micos](https://www.pi-usa.us) controllers: MoCo dc controller, SMC hydra controller, SMC corvus controller

[![Build Status](https://github.com/epics-motor/motorMicos/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorMicos/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorMicos.png)](https://travis-ci.org/epics-motor/motorMicos)-->

motorMicos is a submodule of [motor](https://github.com/epics-modules/motor).  When motorMicos is built in the ``motor/modules`` directory, no manual configuration is needed.

motorMicos can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorMicos contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
