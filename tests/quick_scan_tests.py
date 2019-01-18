# TODO: add cv methods

# check if program reaches completion
def test_reach_completion():
    quick_scan()

if __name__ == "__main__":
    # TODO don't import at runtime
    import sys
    sys.path.append('./')
    sys.path.append('../')
    from quick_scan import *
    from autonomy import *

    print("\n##### Test that program completes #####")
    test_reach_completion()
