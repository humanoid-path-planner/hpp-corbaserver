# Import all CORBA IDLs.
import hpp_corbaserver.hpp

from hpp.corbaserver.client import *

def parseOptions ():
    '''Parse command line options.'''
    from optparse import OptionParser
    usage = "usage: %prog [-p N]"
    parser = OptionParser (usage=usage)
    parser.add_option ("-p", "--problem", dest="problem",
                       help="solve only one problem", type="int")

    (options, args) = parser.parse_args ()
    return options.problem
