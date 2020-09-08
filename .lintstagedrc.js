module.exports = {
	"*": [
		"eslint --cache --fix --ignore-path .lintignore",
		"prettier --write --ignore-path .lintignore --ignore-unknown",
		(files) => "npm run build",
		(files) => "npm run test",
	],
};
