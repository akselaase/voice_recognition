rm -r arse_cleaned.bak
mv arse_cleaned arse_cleaned.bak
rsync -az --progress --include='*.wav' --include='*/' --exclude='*' --prune-empty-dirs ai@akselaase.no:voicerec/finished/ arse_raw
# python3 sort_samples.py
