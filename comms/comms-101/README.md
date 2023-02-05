# Running http folder program
Open your command terminal of choice and `cd` into the 'http' directory. Then run the following command:
``` bash
python3 http/webserver.py
```
Upon doing so, you will see the terminal output `Listening for clients on localhost:54000`. Then, search for `localhost:54000` in a browser, and a webpage with content will be loaded. Open the browser console (press Ctrl + Shift + I) to view the message that is printed by `index.js`.

Functionality is in place to load another page should you provide that as a parameter in the URL you visit. Try visiting `localhost:54000/other_path.html` to see this in action.

We also added a few `print` statements that output useful nuts-and-bolts content in the command terminal. Feel free to investigate this at your own leisure.