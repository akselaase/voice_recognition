rm -r arse_sorted.bak
mv arse_sorted arse_sorted.bak
rsync -avz ai@akselaase.no:voicerec/finished/ arse_raw
# python3 sort_samples.py
