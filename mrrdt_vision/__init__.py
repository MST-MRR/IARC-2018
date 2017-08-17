############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'Initializes the mrrdt-vision module'

import sys
from .detect import detect_object, ObjectTypes

sys.modules['mrrdt-vision'] = sys.modules[__name__]