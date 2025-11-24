"""
Helper script to check which Python Webots is using.
Run this from Webots console or check Webots preferences.
"""
import sys
import os

print("=" * 60)
print("Python Information for Webots")
print("=" * 60)
print(f"Python executable: {sys.executable}")
print(f"Python version: {sys.version}")
print(f"Python path: {sys.path}")
print("=" * 60)
print("\nTo install numpy, run:")
print(f'"{sys.executable}" -m pip install numpy')


