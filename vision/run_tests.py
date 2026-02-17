import unittest
import os

def main():
    tests_dir = os.path.join(os.path.dirname(__file__), "tests")

    loader = unittest.TestLoader()
    suite = loader.discover(start_dir=tests_dir, pattern="test_*.py")

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    raise SystemExit(not result.wasSuccessful())

if __name__ == "__main__":
    main()
