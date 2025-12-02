import os
from pathlib import Path
from datetime import datetime

def delete_files_by_date(directory_path, target_date_str, dry_run=True):
    """
    Deletes files from a specified directory that were created on a given date.

    Args:
        directory_path (str): The path to the directory to process.
        target_date_str (str): The target date in 'YYYY-MM-DD' format.
        dry_run (bool): If True, lists files to be deleted without deleting them.
                        If False, proceeds with deletion.
    """
    try:
        target_date = datetime.strptime(target_date_str, '%Y-%m-%d').date()
    except ValueError:
        print(f"Error: Invalid date format. Please use 'YYYY-MM-DD'.")
        return

    target_dir = Path(directory_path)

    if not target_dir.is_dir():
        print(f"Error: Directory '{directory_path}' not found.")
        return

    print(f"Scanning directory: {target_dir}")
    print(f"Target date for deletion: {target_date_str}\n")
    print("--- Dry Run Results ---" if dry_run else "--- Deletion Results ---")

    for file_path in target_dir.iterdir():
        if file_path.is_file():
            try:
                # Get the creation time of the file
                # Note: On Unix-like systems, os.path.getctime() might return
                # the time of the last metadata change, not creation.
                # Using the creation time is the best cross-platform option.
                creation_timestamp = os.path.getctime(file_path)
                creation_date = datetime.fromtimestamp(creation_timestamp).date()

                if creation_date == target_date:
                    if dry_run:
                        print(f"[DRY RUN] Would delete: {file_path}")
                    else:
                        os.remove(file_path)
                        print(f"Deleted: {file_path}")

            except (OSError, PermissionError) as e:
                print(f"Could not access or delete file {file_path}: {e}")

    if dry_run:
        print("\nThis was a dry run. To perform the actual deletion, set `dry_run=False`.")
    else:
        print("\nDeletion process complete.")

# Here is how I use it:
delete_files_by_date("./", "2025-10-25", True) # dry run TRUE lists files to be deleted ! 
#delete_files_by_date("./", "2025-11-30", False) # dry run FALSE deletes the files ! 

# caveat : setting the date for the date this file was created will also delete this file.
# USE WITH CAUTION
