import os
import shutil
import tempfile
import sys
import unittest
import pytest

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Visualize_SP_Metric.box_plot_utils import get_subfolder_path, clear_folder

class TestBoxPlotUtils(unittest.TestCase):

    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_get_subfolder_path_valid(self):
        # Create a single subfolder
        subfolder = os.path.join(self.temp_dir, "sub1")
        os.makedirs(subfolder)
        
        # Verify it returns the subfolder path
        res = get_subfolder_path(self.temp_dir)
        self.assertEqual(res, subfolder)

    def test_get_subfolder_path_not_dir(self):
        # Pass a non-existent path
        non_existent = os.path.join(self.temp_dir, "does_not_exist")
        with self.assertRaises(ValueError):
            get_subfolder_path(non_existent)

    def test_get_subfolder_path_multiple_subfolders(self):
        # Create two subfolders
        os.makedirs(os.path.join(self.temp_dir, "sub1"))
        os.makedirs(os.path.join(self.temp_dir, "sub2"))
        
        with self.assertRaises(ValueError) as ctx:
            get_subfolder_path(self.temp_dir)
        self.assertIn("more than one subfolder", str(ctx.exception))

    def test_get_subfolder_path_no_subfolders(self):
        # Keep directory empty, or containing only files
        with open(os.path.join(self.temp_dir, "file.txt"), "w") as f:
            f.write("hello")
            
        with self.assertRaises(ValueError) as ctx:
            get_subfolder_path(self.temp_dir)
        self.assertIn("does not contain any subfolders", str(ctx.exception))

    def test_clear_folder_valid(self):
        # Add some files and a subfolder
        file1 = os.path.join(self.temp_dir, "file1.txt")
        file2 = os.path.join(self.temp_dir, "file2.txt")
        subfolder = os.path.join(self.temp_dir, "subdir")
        
        with open(file1, "w") as f:
            f.write("data")
        with open(file2, "w") as f:
            f.write("data")
        os.makedirs(subfolder)
        
        # Call clear_folder
        clear_folder(self.temp_dir)
        
        # Files should be deleted, subfolder should remain
        self.assertFalse(os.path.exists(file1))
        self.assertFalse(os.path.exists(file2))
        self.assertTrue(os.path.exists(subfolder))

    def test_clear_folder_non_existent(self):
        # Should exit gracefully without throwing an exception
        clear_folder(os.path.join(self.temp_dir, "does_not_exist"))
