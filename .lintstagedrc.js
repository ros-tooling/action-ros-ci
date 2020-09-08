module.exports = {
	"*": [
		"eslint --cache --fix --ignore-path .lintignore",
		"prettier --write --ignore-path .lintignore --ignore-unknown",
		// Note: doing the build here ensures we omit unstaged changes
		() => "npm run build",
		() => "git add dist/index.js",
		// same here. Only test staged changes.
		() => "npm run test",
	],
};
