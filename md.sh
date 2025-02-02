#!/bin/bash

# Check if a directory path is provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 <directory_path> [--ignore extension1,extension2,...]"
    exit 1
fi

directory="$1"
ignored_extensions=""

# Parse command line arguments for --ignore flag
if [ $# -gt 1 ] && [ "$2" = "--ignore" ] && [ -n "$3" ]; then
    ignored_extensions="$3"
fi

# Check if directory exists
if [ ! -d "$directory" ]; then
    echo "Error: Directory '$directory' does not exist"
    exit 1
fi

# Create output filename based on directory name
output_file="${directory//\//_}.md"

# Write directory name as heading
echo "# $directory" > "$output_file"

# Function to determine if file should be ignored based on extension
should_ignore() {
    local file="$1"
    local extension="${file##*.}"
    if [ -n "$ignored_extensions" ]; then
        echo "$ignored_extensions" | tr ',' '\n' | grep -q "^$extension$"
        return $?
    fi
    return 1
}

# Process each file in the directory
find "$directory" -type f -maxdepth 1 | sort | while read -r file; do
    filename=$(basename "$file")
    
    # Skip if file should be ignored
    if should_ignore "$filename"; then
        continue
    fi
    
    # Get file extension for language highlighting
    extension="${filename##*.}"
    
    # Determine language for code block
    case "$extension" in
        cpp|ino|h) lang="cpp" ;;
        py) lang="python" ;;
        js) lang="javascript" ;;
        *) lang="" ;;
    esac
    
    # Add filename and contents to markdown
    echo -e "\n$filename:" >> "$output_file"
    
    if [ -n "$lang" ]; then
        echo -e "\`\`\`$lang" >> "$output_file"
    else
        echo -e "\`\`\`" >> "$output_file"
    fi
    
    cat "$file" >> "$output_file"
    echo -e "\`\`\`\n" >> "$output_file"
done

echo "Generated markdown file: $output_file" 