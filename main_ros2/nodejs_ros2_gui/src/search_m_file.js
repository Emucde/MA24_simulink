const fs = require('fs');

// Function to search for trajectory names in a .m file
function searchTrajectoryNames(filePath) {
    return new Promise((resolve, reject) => {
        // Read the contents of the .m file
        fs.readFile(filePath, 'utf8', (err, data) => {
            if (err) {
                reject(`Error reading file: ${err}`);
                return;
            }
            
            // Regular expression to match trajectory names
            const regex = /traj_struct\.name\s*=\s*['"]([^'"]+)['"]/g;
            let matches;
            const trajectoryNames = [];
            let cnt = 1;

            // Find all matches
            while ((matches = regex.exec(data)) !== null) {
                trajectoryNames.push("Traj " + cnt++ + ": " + matches[1]); // Capture group containing the name
            }

            // Resolve with the found trajectory names
            resolve(trajectoryNames);
        });
    });
}

// Example usage
// searchTrajectoryNames('path/to/your/file.m');

module.exports = { searchTrajectoryNames };